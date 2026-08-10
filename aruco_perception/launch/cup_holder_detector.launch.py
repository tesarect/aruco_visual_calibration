"""Launches cup_holder_detector_node, sim-only.

Uses the same env-parameterized pattern as aruco_detector.launch.py (env arg
-> params filename), but only a cup_holder_detector_sim.yaml params file
exists — see cup_holder_detector_node.hpp for why this node has no
real-robot role. env:=real is not a supported/tested path; the argument
exists only for parity with this package's other launch files.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    env_arg = DeclareLaunchArgument(
        "env",
        default_value="sim",
        description="Which parameter file to load: 'sim' (only supported value today)",
    )

    params_filename = PythonExpression(
        ["'cup_holder_detector_' + '", LaunchConfiguration("env"), "' + '.yaml'"]
    )

    params_file = PathJoinSubstitution([
        FindPackageShare("aruco_perception"),
        "config",
        params_filename,
    ])

    # use_sim_time must match the environment — see visual_calibration_moveit's
    # trajectory_planner.launch.py for why (Gazebo-only /clock vs wall time).
    use_sim_time = PythonExpression(["'", LaunchConfiguration("env"), "' == 'sim'"])

    cup_holder_detector_node = Node(
        package="aruco_perception",
        executable="cup_holder_detector_node",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        env_arg,
        cup_holder_detector_node,
    ])
