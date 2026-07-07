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

    trajectory_planner_node = Node(
        package="visual_calibration_moveit",
        executable="trajectory_planner",
        output="screen",
        parameters=[params_file],
    )

    return LaunchDescription([
        env_arg,
        trajectory_planner_node,
    ])
