from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import SetParameter
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    # use_sim_time is required here: Gazebo publishes /joint_states
    # timestamped against sim time (via /clock), not wall time. Without
    # this, move_group's current_state_monitor compares incoming
    # /joint_states timestamps against wall-clock "now" and treats every
    # message as stale, no matter how fast Gazebo is actually publishing —
    # symptom: "Didn't received robot state (joint angles) with recent
    # timestamp", planning succeeds (doesn't need live state) but
    # execution always aborts (validates against current state first).
    # This package was copied from the instructor's real-robot-only
    # ur3e_moveit_config (see package.xml), which correctly has no
    # use_sim_time override — real has no /clock and must stay on wall
    # time. See aruco_detector.launch.py for the same use_sim_time pattern
    # applied via an env:= arg.
    #
    # SetParameter inside a GroupAction applies use_sim_time to every node
    # launched within the group, regardless of how generate_move_group_launch
    # assembled its LaunchDescription internally — more robust than trying
    # to mutate individual entities' parameters after the fact.
    moveit_config = (
        MoveItConfigsBuilder("name", package_name="sim_ur3e_moveit_config")
        .to_moveit_configs()
    )
    move_group_ld = generate_move_group_launch(moveit_config)
    return LaunchDescription([
        GroupAction([
            SetParameter(name="use_sim_time", value=True),
            move_group_ld,
        ]),
    ])
