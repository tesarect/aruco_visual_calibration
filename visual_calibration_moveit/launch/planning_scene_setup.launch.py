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

    # LaunchConfiguration only resolves to a string at launch time, so the
    # filename can't be built with plain Python str concatenation or
    # os.path.join() here — PythonExpression evaluates the concatenation
    # once "env" is known. PathJoinSubstitution can't do this part either:
    # it always inserts '/' between elements, which would corrupt the
    # filename instead of just gluing "scene_objects_" + env + ".yaml".
    params_filename = PythonExpression(
        ["'scene_objects_' + '", LaunchConfiguration("env"), "' + '.yaml'"]
    )

    params_file = PathJoinSubstitution([
        FindPackageShare("visual_calibration_moveit"),
        "config",
        params_filename,
    ])

    planning_scene_setup_node = Node(
        package="visual_calibration_moveit",
        executable="planning_scene_setup",
        output="screen",
        parameters=[params_file],
    )

    return LaunchDescription([
        env_arg,
        planning_scene_setup_node,
    ])