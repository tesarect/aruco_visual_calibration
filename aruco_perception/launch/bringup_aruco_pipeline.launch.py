"""NEW, additive file — does not replace or modify aruco_detector.launch.py
or calibration_broadcaster.launch.py, both left untouched and still
independently launchable. See visual_calibration_moveit/launch/
bringup_full_sim_README.md (and bringup_full_real_README.md) for the full
staged-bringup design this file is part of.

Combines aruco_detector_node + calibration_broadcaster_node into one launch
file, gated on the same condition sim_tmux_trajcal.sh/real_tmux_trajcal.sh
chain today via `wait_for_node.sh aruco_detector_node` before starting
calibration_broadcaster_node (it needs marker_pose to be publishable).
Reimplements that check via rclpy's get_node_names() (the same mechanism
`ros2 node list` itself uses) rather than shelling out to the .sh script,
for the same install-space-reachability reason as
bringup_moveit_pipeline.launch.py.
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
    elapses. Same substring-match semantics as wait_for_node.sh (`ros2
    node list | grep -q "$NODE_NAME"`). Returns True if found, False on
    timeout — caller decides whether to proceed anyway.
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
            "starting calibration_broadcaster_node anyway (same 'continuing "
            "anyway' convention as wait_for_node.sh).", flush=True,
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
        # See bringup_moveit_pipeline.launch.py's matching comment: this
        # only guarantees aruco_detector_node's process is STARTED before
        # the wait begins — the blocking wait inside _launch_setup is what
        # actually enforces the gate.
        OpaqueFunction(function=_launch_setup),
    ])
