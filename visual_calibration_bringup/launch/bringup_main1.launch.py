"""ROS-native equivalent of resources/scripts/tmux/sim_tmux_main1.sh:
trajectory_planner, aruco_detector_node, calibration_broadcaster_node.
Assumes bringup_base.launch.py (sim, move_group, planning scene) is
already running — this file does not start those itself.

Sequencing: trajectory_planner needs move_group (MoveGroupInterface);
calibration_broadcaster_node needs aruco_detector_node's marker_pose topic
and trajectory_planner's ~/trace_polygon service to be callable. Each
dependency is polled via wait_for_node_action and chained with
RegisterEventHandler + OnProcessExit — see bringup_base.launch.py's
docstring for why a poll (not a true OnProcessStart handler) is used, and
why listing actions in one LaunchDescription does not itself sequence
them.

Starting calibration itself (calling ~/start_calibration and
~/trace_polygon) is left as a manual step — see README.md's Manual start
sequence — since it's a deliberate user action, not part of bringup.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from wait_for_node_action import wait_for_node_action


def generate_launch_description():
    wait_for_move_group = wait_for_node_action("move_group", timeout_sec=30)

    trajectory_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("visual_calibration_moveit"), "launch",
                "trajectory_planner.launch.py",
            ])
        ),
        launch_arguments={"env": "sim"}.items(),
    )

    wait_for_trajectory_planner = wait_for_node_action("trajectory_planner", timeout_sec=30)

    aruco_detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("aruco_perception"), "launch", "aruco_detector.launch.py",
            ])
        ),
        launch_arguments={"env": "sim"}.items(),
    )

    wait_for_aruco_detector = wait_for_node_action("aruco_detector_node", timeout_sec=30)

    calibration_broadcaster = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("aruco_perception"), "launch",
                "calibration_broadcaster.launch.py",
            ])
        ),
        launch_arguments={"env": "sim"}.items(),
    )

    start_trajectory_planner_after_move_group = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_move_group,
            on_exit=[trajectory_planner, wait_for_trajectory_planner],
        )
    )
    start_aruco_detector_after_trajectory_planner = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_trajectory_planner,
            on_exit=[aruco_detector, wait_for_aruco_detector],
        )
    )
    start_calibration_broadcaster_after_aruco_detector = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_aruco_detector,
            on_exit=[calibration_broadcaster],
        )
    )

    return LaunchDescription([
        wait_for_move_group,
        start_trajectory_planner_after_move_group,
        start_aruco_detector_after_trajectory_planner,
        start_calibration_broadcaster_after_aruco_detector,
    ])