"""NEW, additive file — does not replace or modify calibration_orchestrator.launch.py,
which is left untouched and still independently launchable. See
visual_calibration_moveit/launch/bringup_full_sim_README.md (and
bringup_full_real_README.md) for the full staged-bringup design this file
is part of.

Cross-package: includes visual_calibration_moveit's bringup_moveit_pipeline
(planning_scene_setup -> trajectory_planner, scene-content gated) and
aruco_perception's bringup_aruco_pipeline (aruco_detector_node ->
calibration_broadcaster_node, node-presence gated), THEN gates
calibration_orchestrator_node on calibration_broadcaster_node AND
trajectory_planner both being up — mirrors sim_tmux_trajcal.sh/
real_tmux_trajcal.sh's pane 3 chain (`wait_for_node.sh
calibration_broadcaster_node && wait_for_node.sh trajectory_planner`).

This is the file that, once trusted, replaces the largest chunk of
sim_tmux_trajcal.sh/real_tmux_trajcal.sh's pane count in one shot — it
transitively brings up planning_scene_setup, trajectory_planner,
aruco_detector_node, calibration_broadcaster_node, AND
calibration_orchestrator_node from a single `ros2 launch` call.
"""

import time

import rclpy
from rclpy.node import Node as RclpyNode

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


NODE_WAIT_TIMEOUT_SEC = 30.0
NODE_WAIT_POLL_INTERVAL_SEC = 1.0


def _wait_for_nodes(node_name_substrings, timeout_sec, interval_sec):
    """Waits for EVERY name in node_name_substrings to appear in the ROS
    graph (substring match, same semantics as wait_for_node.sh), not just
    the first. Returns True only if all are found before timeout_sec.
    """
    rclpy.init(args=None)
    node = RclpyNode("bringup_orchestrator_pipeline_node_wait")

    deadline = time.monotonic() + timeout_sec
    pending = set(node_name_substrings)
    while time.monotonic() < deadline and pending:
        names = node.get_node_names()
        pending = {sub for sub in pending if not any(sub in n for n in names)}
        if not pending:
            break
        time.sleep(interval_sec)

    node.destroy_node()
    rclpy.shutdown()
    return not pending


def _launch_setup(context, *args, **kwargs):
    env = LaunchConfiguration("env").perform(context)

    required = ["calibration_broadcaster_node", "trajectory_planner"]
    print(
        f"[bringup_orchestrator_pipeline] Waiting for {required} "
        f"(timeout {NODE_WAIT_TIMEOUT_SEC:.0f}s)...", flush=True,
    )
    ready = _wait_for_nodes(required, NODE_WAIT_TIMEOUT_SEC, NODE_WAIT_POLL_INTERVAL_SEC)
    if ready:
        print(
            "[bringup_orchestrator_pipeline] Dependencies up — starting "
            "calibration_orchestrator_node.", flush=True,
        )
    else:
        print(
            "[bringup_orchestrator_pipeline] Timed out waiting for dependencies — "
            "starting calibration_orchestrator_node anyway (same 'continuing "
            "anyway' convention as wait_for_node.sh).", flush=True,
        )

    calibration_orchestrator_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("orchestrator"),
                "launch", "calibration_orchestrator.launch.py",
            ])
        ),
        launch_arguments={"env": env}.items(),
    )
    return [calibration_orchestrator_include]


def generate_launch_description():
    env_arg = DeclareLaunchArgument(
        "env",
        default_value="sim",
        description="Which parameter file to load: 'sim' or 'real'",
    )

    moveit_pipeline_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("visual_calibration_moveit"),
                "launch", "bringup_moveit_pipeline.launch.py",
            ])
        ),
        launch_arguments={"env": LaunchConfiguration("env")}.items(),
    )

    aruco_pipeline_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("aruco_perception"),
                "launch", "bringup_aruco_pipeline.launch.py",
            ])
        ),
        launch_arguments={"env": LaunchConfiguration("env")}.items(),
    )

    return LaunchDescription([
        env_arg,
        moveit_pipeline_include,
        aruco_pipeline_include,
        # See bringup_moveit_pipeline.launch.py's matching comment: this
        # only guarantees both included pipelines' processes are STARTED
        # before the wait begins — the blocking wait inside _launch_setup
        # is what actually enforces the gate.
        OpaqueFunction(function=_launch_setup),
    ])
