"""Combines aruco_detector_node and calibration_broadcaster_node into a single
launch file. aruco_detector.launch.py and calibration_broadcaster.launch.py
remain independently launchable.

calibration_broadcaster_node needs marker_pose to be publishable, so its
start is gated on aruco_detector_node already being up in the ROS graph.
The wait is implemented via rclpy's get_node_names() (the same mechanism
`ros2 node list` uses) rather than shelling out to a script, so it works
directly from the install space.
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


def _wait_for_node(node_name_substring, timeout_sec, interval_sec):
    """Blocks the launch process until a node whose fully-qualified name
    contains node_name_substring appears in the ROS graph, or timeout_sec
    elapses. Returns True if found, False on timeout — caller decides
    whether to proceed anyway.
    """
    rclpy.init(args=None)
    node = RclpyNode("bringup_aruco_pipeline_node_wait")

    deadline = time.monotonic() + timeout_sec
    found = False
    while time.monotonic() < deadline:
        names = node.get_node_names()
        if any(node_name_substring in n for n in names):
            found = True
            break
        time.sleep(interval_sec)

    node.destroy_node()
    rclpy.shutdown()
    return found


def _launch_setup(context, *args, **kwargs):
    env = LaunchConfiguration("env").perform(context)

    print(
        f"[bringup_aruco_pipeline] Waiting for aruco_detector_node "
        f"(timeout {NODE_WAIT_TIMEOUT_SEC:.0f}s)...", flush=True,
    )
    ready = _wait_for_node("aruco_detector_node", NODE_WAIT_TIMEOUT_SEC, NODE_WAIT_POLL_INTERVAL_SEC)
    if ready:
        print(
            "[bringup_aruco_pipeline] aruco_detector_node is up — starting "
            "calibration_broadcaster_node.", flush=True,
        )
    else:
        print(
            "[bringup_aruco_pipeline] Timed out waiting for aruco_detector_node — "
            "starting calibration_broadcaster_node anyway.", flush=True,
        )

    calibration_broadcaster_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("aruco_perception"),
                "launch", "calibration_broadcaster.launch.py",
            ])
        ),
        launch_arguments={"env": env}.items(),
    )
    return [calibration_broadcaster_include]


def generate_launch_description():
    env_arg = DeclareLaunchArgument(
        "env",
        default_value="sim",
        description="Which parameter file to load: 'sim' or 'real'",
    )

    aruco_detector_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("aruco_perception"),
                "launch", "aruco_detector.launch.py",
            ])
        ),
        launch_arguments={"env": LaunchConfiguration("env")}.items(),
    )

    return LaunchDescription([
        env_arg,
        aruco_detector_include,
        # Only guarantees aruco_detector_node's process is STARTED before the
        # wait begins — the blocking wait inside _launch_setup is what
        # actually enforces the gate.
        OpaqueFunction(function=_launch_setup),
    ])
