"""NEW, additive file — does not replace or modify planning_scene_setup.launch.py
or trajectory_planner.launch.py, both left untouched and still independently
launchable. See visual_calibration_moveit/launch/bringup_full_sim_README.md
(and bringup_full_real_README.md) for the full staged-bringup design this
file is part of.

Combines planning_scene_setup + trajectory_planner into one launch file,
gated on the SAME condition resources/scripts/python/wait_for_planning_scene.py
checks today (that script isn't installed/reachable from an install-space
launch file, so its /get_planning_scene polling logic is reimplemented
inline below rather than shelled out to) — trajectory_planner's
move_to_home_on_startup plans/executes as soon as its constructor runs, and
collision checking can only avoid obstacles already IN the scene at that
moment. planning_scene_setup does not exit after populating the scene
(rclcpp::spin() runs immediately after construction, confirmed in
src/planning_scene_setup/main.cpp) — so neither "is the node in ros2 node
list" nor "did the process exit" is a valid readiness signal here; only the
scene's actual contents are.

Does NOT wait for move_group itself — both included launch files' own nodes
(planning_scene_setup via PlanningSceneInterface, trajectory_planner via
MoveGroupInterface) already block on move_group internally on first use, the
same as running them via `ros2 launch` directly today.

Real-only, additionally: also waits for scaled_joint_trajectory_controller
to report "active" on /controller_manager/list_controllers before starting
trajectory_planner — see REQUIRED_CONTROLLER_NAME's doc comment below for
the live failure this closes (move_group/planning-scene readiness does not
imply the real robot's controller connection has finished activating).
"""

import time

import rclpy
from rclpy.node import Node as RclpyNode

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


REQUIRED_SCENE_OBJECTS = {"countertop", "wall"}
SCENE_POLL_TIMEOUT_SEC = 30.0
SCENE_POLL_INTERVAL_SEC = 2.0

# Real-robot only (see _launch_setup) — added 2026-07-23 after a live
# bringup_moveit_pipeline.launch.py test on real hit "Goal was rejected by
# server" from scaled_joint_trajectory_controller on trajectory_planner's
# very first move_to_home_on_startup call. move_group being present in
# `ros2 node list` (or even successfully serving /get_planning_scene) does
# NOT mean its controller_manager connection has finished activating
# scaled_joint_trajectory_controller — the existing tmux-script flow never
# hit this because starting each session by hand left enough incidental
# delay for the controller to settle before trajectory_planner started;
# this bringup chain runs faster/more automated and can race ahead of
# that settle time. Same fix/convention as resources/scripts/shell/
# wait_for_controllers.sh (generalized the same day to accept any
# controller name, not just joint_state_broadcaster) — reimplemented here
# via controller_manager_msgs/ListControllers directly (same reasoning as
# _wait_for_planning_scene: the shell script isn't installed/reachable
# from an install-space launch file).
REQUIRED_CONTROLLER_NAME = "scaled_joint_trajectory_controller"
CONTROLLER_POLL_TIMEOUT_SEC = 60.0
CONTROLLER_POLL_INTERVAL_SEC = 1.0


def _wait_for_controller_active(timeout_sec, interval_sec):
    """Blocks the launch process itself until REQUIRED_CONTROLLER_NAME
    reports state "active" on /controller_manager/list_controllers, or
    timeout_sec elapses. One persistent client, not a respawned CLI call
    per poll — same pattern _wait_for_planning_scene already uses, for
    the same "avoid client connect/disconnect churn against a shared ROS
    service" reason documented in wait_for_planning_scene.py's docstring.
    Returns True if satisfied, False on timeout.
    """
    from controller_manager_msgs.srv import ListControllers

    rclpy.init(args=None)
    node = RclpyNode("bringup_moveit_pipeline_controller_wait")
    client = node.create_client(ListControllers, "/controller_manager/list_controllers")
    client.wait_for_service()

    deadline = time.monotonic() + timeout_sec
    satisfied = False
    while time.monotonic() < deadline:
        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        result = future.result()
        states = {c.name: c.state for c in result.controller} if result else {}
        if states.get(REQUIRED_CONTROLLER_NAME) == "active":
            satisfied = True
            break
        time.sleep(interval_sec)

    node.destroy_node()
    rclpy.shutdown()
    return satisfied


def _wait_for_planning_scene(timeout_sec, interval_sec):
    """Blocks the launch process itself (not a ROS node's callback) until
    /get_planning_scene reports both REQUIRED_SCENE_OBJECTS present, or
    timeout_sec elapses. Mirrors wait_for_planning_scene.py's polling
    approach exactly (one persistent client, not a respawned CLI call per
    poll — see that script's docstring for why the respawn approach was
    abandoned). Returns True if satisfied, False on timeout — caller
    decides whether to proceed anyway (same convention as the shell
    scripts this replaces).
    """
    from moveit_msgs.srv import GetPlanningScene
    from moveit_msgs.msg import PlanningSceneComponents

    rclpy.init(args=None)
    node = RclpyNode("bringup_moveit_pipeline_scene_wait")
    client = node.create_client(GetPlanningScene, "/get_planning_scene")
    client.wait_for_service()

    deadline = time.monotonic() + timeout_sec
    satisfied = False
    while time.monotonic() < deadline:
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        result = future.result()
        known = {obj.id for obj in result.scene.world.collision_objects} if result else set()
        if REQUIRED_SCENE_OBJECTS.issubset(known):
            satisfied = True
            break
        time.sleep(interval_sec)

    node.destroy_node()
    rclpy.shutdown()
    return satisfied


def _launch_setup(context, *args, **kwargs):
    env = LaunchConfiguration("env").perform(context)

    print(
        f"[bringup_moveit_pipeline] Waiting for planning scene to contain "
        f"{sorted(REQUIRED_SCENE_OBJECTS)} (timeout {SCENE_POLL_TIMEOUT_SEC:.0f}s)...",
        flush=True,
    )
    ready = _wait_for_planning_scene(SCENE_POLL_TIMEOUT_SEC, SCENE_POLL_INTERVAL_SEC)
    if ready:
        print(
            "[bringup_moveit_pipeline] Planning scene populated — starting "
            "trajectory_planner.", flush=True,
        )
    else:
        print(
            "[bringup_moveit_pipeline] Timed out waiting for planning scene — "
            "starting trajectory_planner anyway (same 'continuing anyway' "
            "convention as wait_for_planning_scene.sh).", flush=True,
        )

    # Real only — see REQUIRED_CONTROLLER_NAME's doc comment above for why
    # (this specific "Goal was rejected by server" failure was only ever
    # observed against the real robot's controller_manager; sim's
    # equivalent joint_state_broadcaster-readiness gate already runs
    # earlier, in sim_tmux_base.sh, before move_group itself even starts).
    if env == "real":
        print(
            f"[bringup_moveit_pipeline] Waiting for "
            f"'{REQUIRED_CONTROLLER_NAME}' to be active "
            f"(timeout {CONTROLLER_POLL_TIMEOUT_SEC:.0f}s)...", flush=True,
        )
        controller_ready = _wait_for_controller_active(
            CONTROLLER_POLL_TIMEOUT_SEC, CONTROLLER_POLL_INTERVAL_SEC)
        if controller_ready:
            print(
                f"[bringup_moveit_pipeline] '{REQUIRED_CONTROLLER_NAME}' is "
                "active — starting trajectory_planner.", flush=True,
            )
        else:
            print(
                f"[bringup_moveit_pipeline] Timed out waiting for "
                f"'{REQUIRED_CONTROLLER_NAME}' to activate — starting "
                "trajectory_planner anyway (same 'continuing anyway' "
                "convention as the other waits above).", flush=True,
            )

    trajectory_planner_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("visual_calibration_moveit"),
                "launch", "trajectory_planner.launch.py",
            ])
        ),
        launch_arguments={"env": env}.items(),
    )
    return [trajectory_planner_include]


def generate_launch_description():
    env_arg = DeclareLaunchArgument(
        "env",
        default_value="sim",
        description="Which parameter file to load: 'sim' or 'real'",
    )

    planning_scene_setup_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("visual_calibration_moveit"),
                "launch", "planning_scene_setup.launch.py",
            ])
        ),
        launch_arguments={"env": LaunchConfiguration("env")}.items(),
    )

    return LaunchDescription([
        env_arg,
        planning_scene_setup_include,
        # OpaqueFunction runs at launch-processing time, AFTER the action
        # above has been added to the launch description — it does NOT
        # block waiting for planning_scene_setup's node to finish adding
        # objects before running. The blocking wait inside _launch_setup
        # (_wait_for_planning_scene) is what actually enforces the gate;
        # this ordering only guarantees planning_scene_setup's process is
        # STARTED before the wait begins, same guarantee wait_for_node.sh
        # move_group gave sim_tmux_trajcal.sh before it additionally
        # chained wait_for_planning_scene.sh.
        OpaqueFunction(function=_launch_setup),
    ])
