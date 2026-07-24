from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Standalone launch file for yolo_marker_bridge_node -- mirrors
    aruco_perception/launch/aruco_detector.launch.py's env:=sim|real
    pattern exactly, so switching between the classical detector and this
    YOLO-backed alternative is just a matter of which launch file is
    invoked (never both at once -- both publish on the same pose_topic).

    NOT wired into any combined bringup launch file yet (e.g.
    visual_calibration_bringup's bringup_main1.launch.py) -- that's a
    follow-up coordination point for a ROS-focused pass, not done here
    per this task's scope (no edits to existing packages' launch/CMake
    wiring).
    """
    env_arg = DeclareLaunchArgument(
        "env",
        default_value="sim",
        description="Which parameter file to load: 'sim' or 'real'",
    )

    params_filename = PythonExpression(
        ["'yolo_marker_bridge_' + '", LaunchConfiguration("env"), "' + '.yaml'"]
    )

    params_file = PathJoinSubstitution([
        FindPackageShare("aruco_perception_yolo_bridge"),
        "config",
        params_filename,
    ])

    # use_sim_time must match the environment -- same reasoning as
    # aruco_detector.launch.py / trajectory_planner.launch.py (Gazebo-only
    # /clock vs wall time -- see error-mitigation.md #16).
    use_sim_time = PythonExpression(["'", LaunchConfiguration("env"), "' == 'sim'"])

    yolo_marker_bridge_node = Node(
        package="aruco_perception_yolo_bridge",
        executable="yolo_marker_bridge_node.py",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        env_arg,
        yolo_marker_bridge_node,
    ])