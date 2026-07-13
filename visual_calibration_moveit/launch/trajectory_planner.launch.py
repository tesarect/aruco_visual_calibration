from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    env_arg = DeclareLaunchArgument(
        "env",
        default_value="sim",
        description="Which parameter file to load: 'sim' or 'real'",
    )

    params_filename = PythonExpression(
        ["'trajectory_planner_' + '", LaunchConfiguration("env"), "' + '.yaml'"]
    )

    params_file = PathJoinSubstitution([
        FindPackageShare("visual_calibration_moveit"),
        "config",
        params_filename,
    ])

    # Second params file, same node namespace (trajectory_planner) — see
    # preset_poses_sim.yaml/_real.yaml. ROS 2 merges multiple `parameters`
    # entries for the same node, so preset_names/<name>.position/
    # <name>.orientation land alongside camera_frame/standoff_m/etc.
    preset_poses_filename = PythonExpression(
        ["'preset_poses_' + '", LaunchConfiguration("env"), "' + '.yaml'"]
    )

    preset_poses_file = PathJoinSubstitution([
        FindPackageShare("visual_calibration_moveit"),
        "config",
        preset_poses_filename,
    ])

    # use_sim_time must match the environment: Gazebo publishes /clock on
    # sim time in env:=sim, while env:=real has no simulated clock at all.
    # This node calls get_clock()->now() and does TF lookups, so a mismatch
    # here risks the same class of timing bug as error-mitigation.md #5/#9.
    use_sim_time = PythonExpression(["'", LaunchConfiguration("env"), "' == 'sim'"])

    trajectory_planner_node = Node(
        package="visual_calibration_moveit",
        executable="trajectory_planner",
        output="screen",
        parameters=[params_file, preset_poses_file, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        env_arg,
        trajectory_planner_node,
    ])
