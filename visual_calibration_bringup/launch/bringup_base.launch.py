"""ROS-native equivalent of resources/scripts/tmux/sim_tmux_base.sh:
simulation, move_group, rviz, planning scene. Does not start
trajectory_planner / aruco_detector_node / calibration_broadcaster_node —
see bringup_main1.launch.py for those. Does not start the marker-debugger
(resources/scripts/python/tf_debug_markers.py) either — that's a
standalone debug script outside the ROS package/launch system, run
manually or via the tmux scripts.

Sequencing mirrors the tmux script's own mixed strategy: Gazebo has no
clean topic/service readiness signal, so move_group is gated behind a
fixed TimerAction delay; move_group's own readiness is then checked via
wait_for_node_action's polling loop (see that file — a true OnProcessStart
handler can't attach to move_group directly, since it's included via
IncludeLaunchDescription rather than declared here). What IS a real event
handler: rviz/planning_scene are deferred via RegisterEventHandler +
OnProcessExit, which only fires once the poll process itself exits —
listing actions in the same LaunchDescription would NOT sequence them.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from wait_for_node_action import wait_for_node_action


def generate_launch_description():
    simulation = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("the_construct_office_gazebo"),
                "launch",
                "starbots_ur3e.launch.xml",
            ])
        ),
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("aruco_moveit_config"), "launch", "move_group.launch.py",
            ])
        ),
    )
    # No clean readiness signal exists for "Gazebo fully up" — fixed delay,
    # same as sim_tmux_base.sh's `sleep 5`.
    move_group_after_sim = TimerAction(period=5.0, actions=[move_group])

    wait_for_move_group = wait_for_node_action("move_group", timeout_sec=30)

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("aruco_moveit_config"), "launch", "moveit_rviz.launch.py",
            ])
        ),
    )

    planning_scene = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("visual_calibration_moveit"), "launch",
                "planning_scene_setup.launch.py",
            ])
        ),
        launch_arguments={"env": "sim"}.items(),
    )

    # Actions listed together in one LaunchDescription start concurrently,
    # NOT in sequence — merely listing rviz/planning_scene after
    # wait_for_move_group would not make them wait for it. RegisterEventHandler
    # + OnProcessExit is what actually defers them until the poll process
    # (wait_for_move_group) exits — i.e. move_group confirmed up, or the
    # poll's own timeout elapsed.
    start_after_move_group = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_move_group,
            on_exit=[rviz, planning_scene],
        )
    )

    return LaunchDescription([
        simulation,
        move_group_after_sim,
        wait_for_move_group,
        start_after_move_group,
    ])