from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import SetParameter
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    # use_sim_time is required here: Gazebo publishes /joint_states
    # timestamped against sim time (via /clock), not wall time. Without
    # this, move_group's current_state_monitor compares incoming
    # /joint_states timestamps against wall-clock "now" and treats every
    # message as stale, no matter how fast Gazebo is actually publishing —
    # symptom: "Didn't received robot state (joint angles) with recent
    # timestamp", planning succeeds (doesn't need live state) but
    # execution always aborts (validates against current state first).
    # This package was copied from the instructor's real-robot-only
    # ur3e_moveit_config (see package.xml), which correctly has no
    # use_sim_time override — real has no /clock and must stay on wall
    # time. See aruco_detector.launch.py for the same use_sim_time pattern
    # applied via an env:= arg.
    #
    # SetParameter inside a GroupAction applies use_sim_time to every node
    # launched within the group, regardless of how generate_move_group_launch
    # assembled its LaunchDescription internally — more robust than trying
    # to mutate individual entities' parameters after the fact.
    # .trajectory_execution() must be called explicitly with an explicit
    # file_path — to_moveit_configs() only calls it with file_path=None as
    # a fallback if nothing else already loaded it, and file_path=None
    # searches for *_controllers.yaml (i.e. moveit_controllers.yaml) by
    # itself. Explicitly pointing this at moveit_controllers.yaml (the
    # SAME file auto-discovery would have found) keeps the controller-
    # manager plugin config (moveit_controller_manager,
    # moveit_simple_controller_manager.*) loading correctly — an earlier
    # attempt at this fix pointed file_path at trajectory_execution.yaml
    # instead, which suppressed moveit_controllers.yaml entirely and broke
    # move_group with "Parameter '~moveit_controller_manager' not
    # specified" on real_ur3e_moveit_config (2026-07-18); same gap would
    # apply here.
    #
    # trajectory_execution.yaml's content (allowed_execution_duration_scaling)
    # is instead applied below via SetParameter, alongside use_sim_time —
    # see that comment for why.
    # .planning_pipelines(pipelines=["ompl"], load_all=False) restricts
    # move_group's PlanningPipeline dispatch to OMPL only — necessary but
    # NOT sufficient to keep CHOMP out of the process; see the
    # `capabilities` SetParameter below and
    # real_ur3e_moveit_config/launch/move_group.launch.py's matching
    # comment for the full investigation.
    #
    # DO NOT add "chomp" or "pilz_industrial_motion_planner" here — neither
    # is configured (no chomp_planning.yaml / pilz planning yaml in this
    # package; only pilz_cartesian_limits.yaml exists, which isn't the same
    # thing) or tested against this project's planning group. Only "ompl"
    # is verified — see trajectory_planner_sim.yaml's planning_pipeline_id
    # comment.
    #
    # .pilz_cartesian_limits(file_path=...) must ALSO be called explicitly
    # once .planning_pipelines() is called explicitly — see
    # real_ur3e_moveit_config/launch/move_group.launch.py's matching
    # comment for why (to_dict() unconditionally reads
    # self.pilz_cartesian_limits["robot_description_planning"] with no
    # None-check; omitting this crashed move_group startup with KeyError:
    # 'robot_description_planning', confirmed via `ros2 launch ... --debug`
    # full traceback, 2026-07-19).
    moveit_config = (
        MoveItConfigsBuilder("name", package_name="sim_ur3e_moveit_config")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"], load_all=False)
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .to_moveit_configs()
    )
    move_group_ld = generate_move_group_launch(moveit_config)

    # Overrides MoveIt's execution-duration watchdog budget (default
    # scaling 1.2x — too tight for some of this project's moves, both
    # falsely "aborting" them in sim and actively cancelling a still-
    # executing trajectory mid-motion on real hardware, 2026-07-17/18 — see
    # config/trajectory_execution.yaml's comment for the full
    # investigation). Applied as a launch-time parameter override rather
    # than a second yaml file, since .trajectory_execution() only accepts
    # one file_path and that slot is already used for moveit_controllers.yaml
    # above.
    return LaunchDescription([
        GroupAction([
            SetParameter(name="use_sim_time", value=True),
            SetParameter(
                name="trajectory_execution.allowed_execution_duration_scaling",
                value=4.0),
            # Root cause of the chomp_planner crash: move_group loads the
            # CHOMP planner PLUGIN directly via pluginlib as part of one of
            # its default MoveGroupCapability classes (most likely
            # MoveGroupQueryPlannersService) — completely independent of
            # planning_pipelines, which was already confirmed clean
            # (ompl-only) via a full `ros2 param dump /move_group`.
            # Confirmed via `cat /proc/$(pgrep -f
            # moveit_ros_move_group/move_group)/maps | grep -i chomp`
            # showing libmoveit_chomp_planner_plugin.so genuinely loaded
            # into move_group's own process memory (2026-07-19) — see
            # real_ur3e_moveit_config/launch/move_group.launch.py's
            # matching comment for the full investigation.
            # ros-humble-moveit-planners-chomp is installed system-wide in
            # this rosject, so its plugin is globally discoverable by
            # pluginlib no matter what our own yaml says — we deliberately
            # do not touch that system-wide install, and instead whitelist
            # only the capabilities TrajectoryPlanner actually uses (no
            # /compute_ik, /compute_fk, /check_state_validity,
            # /query_planner_interface, /clear_octomap, or
            # /plan_kinematic_path call anywhere in this codebase).
            SetParameter(
                name="capabilities",
                value=(
                    "move_group/MoveGroupMoveAction "
                    "move_group/MoveGroupExecuteTrajectoryAction "
                    "move_group/MoveGroupCartesianPathService"
                )),
            move_group_ld,
        ]),
    ])
