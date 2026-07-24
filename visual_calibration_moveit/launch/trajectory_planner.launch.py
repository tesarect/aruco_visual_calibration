import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *args, **kwargs):
    env = LaunchConfiguration("env").perform(context)

    # kinematics.yaml's top-level key is "ur_manipulator" (the solver
    # config), but MoveGroupInterface's RobotModelLoader looks for it
    # under the node's robot_description_kinematics.<group> namespace —
    # move_group.launch.py gets this wrapping for free from
    # MoveItConfigsBuilder.robot_description_kinematics(); this node loads
    # the raw yaml itself, so the wrapping has to happen here explicitly
    # (a plain `parameters=[kinematics_file]` load would land the solver
    # config at the WRONG namespace, ur_manipulator.kinematics_solver
    # instead of robot_description_kinematics.ur_manipulator.kinematics_solver,
    # and RobotModelLoader would still report "No kinematics plugins
    # defined" despite the file being loaded without error).
    #
    # Without this, trajectory_planner's own MoveGroupInterface/
    # RobotModelLoader has no kinematics plugin ("No kinematics plugins
    # defined. Fill and load kinematics.yaml!" at startup) and
    # setPoseTarget() can never resolve IK for a Cartesian goal, so every
    # planAndExecute() call (including the startup home move) aborts with
    # "Planning request aborted" / "MoveGroupInterface::plan() failed" —
    # no collision, no CHOMP involved, just no way to convert the pose
    # target into joint values at all (found 2026-07-19, after ruling out
    # planner/pipeline config as the cause — see move_group.launch.py's
    # capabilities comment for that separate investigation).
    # move_group's own move_group.launch.py loads this correctly already
    # (via MoveItConfigsBuilder's default auto-discovery) —
    # trajectory_planner is a SEPARATE node with its own
    # MoveGroupInterface, which needs the same file independently; sourced
    # from {sim,real}_ur3e_moveit_config, same package family as
    # trajectory_planner_{env}.yaml/preset_poses_{env}.yaml below, so env
    # keeps them in sync automatically.
    kinematics_package = "sim_ur3e_moveit_config" if env == "sim" else "real_ur3e_moveit_config"
    kinematics_path = os.path.join(
        get_package_share_directory(kinematics_package), "config", "kinematics.yaml")
    with open(kinematics_path) as f:
        kinematics_solver_config = yaml.safe_load(f)
    robot_description_kinematics = {"robot_description_kinematics": kinematics_solver_config}

    params_filename = "trajectory_planner_" + env + ".yaml"
    params_file = PathJoinSubstitution([
        FindPackageShare("visual_calibration_moveit"), "config", params_filename,
    ])

    # Second params file, same node namespace (trajectory_planner) — see
    # preset_poses_sim.yaml/_real.yaml. ROS 2 merges multiple `parameters`
    # entries for the same node, so preset_names/<name>.position/
    # <name>.orientation land alongside camera_frame/standoff_m/etc.
    preset_poses_filename = "preset_poses_" + env + ".yaml"
    preset_poses_file = PathJoinSubstitution([
        FindPackageShare("visual_calibration_moveit"), "config", preset_poses_filename,
    ])

    # use_sim_time must match the environment: Gazebo publishes /clock on
    # sim time in env:=sim, while env:=real has no simulated clock at all.
    # This node calls get_clock()->now() and does TF lookups, so a mismatch
    # here risks the same class of timing bug as error-mitigation.md #5/#9.
    use_sim_time = (env == "sim")

    trajectory_planner_node = Node(
        package="visual_calibration_moveit",
        executable="trajectory_planner",
        output="screen",
        parameters=[
            params_file,
            preset_poses_file,
            robot_description_kinematics,
            {"use_sim_time": use_sim_time},
        ],
    )

    return [trajectory_planner_node]


def generate_launch_description():
    env_arg = DeclareLaunchArgument(
        "env",
        default_value="sim",
        description="Which parameter file to load: 'sim' or 'real'",
    )

    return LaunchDescription([
        env_arg,
        OpaqueFunction(function=_launch_setup),
    ])