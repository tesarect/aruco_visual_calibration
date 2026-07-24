from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import SetParameter
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
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
    # specified" (2026-07-18).
    #
    # trajectory_execution.yaml's content (allowed_execution_duration_scaling)
    # is instead applied below via SetParameter — see that comment for why.
    # .planning_pipelines(pipelines=["ompl"], load_all=False) restricts
    # move_group's PlanningPipeline dispatch (the planning_pipelines/ompl.*
    # params, what actually plans a request) to OMPL only. This is
    # necessary but NOT sufficient to keep CHOMP out of the process — see
    # the `capabilities` SetParameter below for the other half of this fix
    # (CHOMP got loaded via a MoveGroupCapability's own direct pluginlib
    # call, a completely separate mechanism from planning_pipelines, and
    # this alone did not stop it — confirmed 2026-07-19, full investigation
    # in that comment).
    #
    # DO NOT add "chomp" or "pilz_industrial_motion_planner" to pipelines
    # — neither has a config file in this package (config/chomp_planning.yaml,
    # config/pilz_industrial_motion_planner_planning.yaml don't exist; only
    # pilz_cartesian_limits.yaml does, which is not the same thing), so
    # adding either pipeline ID here without first authoring/testing that
    # config will reintroduce this same failure mode. Only "ompl" has been
    # configured and verified against this project's planning group — see
    # trajectory_planner_real.yaml's matching planning_pipeline_id comment.
    #
    # .pilz_cartesian_limits(file_path=...) must ALSO be called explicitly
    # once .planning_pipelines() is called explicitly — to_moveit_configs()
    # only auto-calls each builder step (joint_limits, pilz_cartesian_limits,
    # trajectory_execution, ...) as a fallback if nothing else in the chain
    # already ran first, and moveit_configs_builder.py's to_dict() (Humble,
    # /opt/ros/humble/lib/python3.10/site-packages/moveit_configs_utils/moveit_configs_builder.py:129)
    # unconditionally reads self.pilz_cartesian_limits["robot_description_planning"]
    # with no None-check — leaving pilz_cartesian_limits unset crashed
    # move_group startup entirely (KeyError: 'robot_description_planning',
    # confirmed via `ros2 launch ... --debug` full traceback, 2026-07-19),
    # unrelated to CHOMP/OMPL despite the similar-looking failure. This
    # points at the SAME pilz_cartesian_limits.yaml to_moveit_configs()
    # would have auto-discovered anyway (see file_path's default in
    # MoveItConfigsBuilder — same reasoning as .trajectory_execution()
    # above).
    moveit_config = (
        MoveItConfigsBuilder("name", package_name="real_ur3e_moveit_config")
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
    # above — SetParameter inside a GroupAction applies it to every node in
    # the group regardless of how generate_move_group_launch assembled its
    # LaunchDescription internally, same pattern sim's move_group.launch.py
    # already uses for use_sim_time.
    return LaunchDescription([
        GroupAction([
            SetParameter(
                name="trajectory_execution.allowed_execution_duration_scaling",
                value=4.0),
            # Root cause of the chomp_planner crash was NOT the pipeline
            # config (planning_pipelines/ompl.* were confirmed clean via
            # `ros2 param dump /move_group` — no "chomp" anywhere) — it was
            # move_group loading the CHOMP planner PLUGIN directly via
            # pluginlib, independent of planning_pipelines, as part of one
            # of its default MoveGroupCapability classes. Confirmed via
            # `cat /proc/$(pgrep -f moveit_ros_move_group/move_group)/maps
            # | grep -i chomp` showing libmoveit_chomp_planner_plugin.so
            # genuinely loaded into move_group's own process memory
            # (2026-07-19) — ros-humble-moveit-planners-chomp is installed
            # system-wide in this rosject (`ros2 pkg prefix
            # moveit_planners_chomp` -> /opt/ros/humble), so its plugin is
            # globally discoverable by pluginlib regardless of our
            # ompl_planning.yaml. move_group's own `capabilities` param
            # defaulted to '' (empty = every default MoveGroupCapability
            # active, confirmed via the same param dump) — one of those
            # (most likely MoveGroupQueryPlannersService, whose own
            # description is "Allow querying of available planners LOADED
            # FROM THE MOTION PLANNING PLUGIN" — see
            # /opt/ros/humble/share/moveit_ros_move_group/default_capabilities_plugin_description.xml)
            # is what's instantiating it.
            #
            # Fix: explicitly whitelist only the capabilities
            # TrajectoryPlanner actually uses (confirmed via grep — no
            # /compute_ik, /compute_fk, /check_state_validity,
            # /query_planner_interface, /clear_octomap, or
            # /plan_kinematic_path call anywhere in this codebase), instead
            # of leaving capabilities as '' (= all defaults, including
            # whichever one eagerly loads every installed planner plugin).
            # We deliberately do NOT touch the system-wide CHOMP package
            # install (e.g. removing its plugin.xml) — this stays a
            # per-launch, reversible parameter, not a change to the shared
            # rosject environment.
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
