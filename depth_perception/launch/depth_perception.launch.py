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
        ["'depth_perception_' + '", LaunchConfiguration("env"), "' + '.yaml'"]
    )

    params_file = PathJoinSubstitution([
        FindPackageShare("depth_perception"),
        "config",
        params_filename,
    ])

    # use_sim_time must match the environment — see visual_calibration_moveit's
    # trajectory_planner.launch.py for why (Gazebo-only /clock vs wall time).
    use_sim_time = PythonExpression(["'", LaunchConfiguration("env"), "' == 'sim'"])

    depth_perception_node = Node(
        package="depth_perception",
        executable="depth_perception_node",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        env_arg,
        depth_perception_node,
    ])
