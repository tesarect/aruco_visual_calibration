from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import SetParameter
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    # use_sim_time — see move_group.launch.py's header comment for why
    # this is needed in sim (Gazebo /clock) but not in the instructor's
    # original real-only ur3e_moveit_config.
    moveit_config = (
        MoveItConfigsBuilder("name", package_name="sim_ur3e_moveit_config")
        .to_moveit_configs()
    )
    rviz_ld = generate_moveit_rviz_launch(moveit_config)
    return LaunchDescription([
        GroupAction([
            SetParameter(name="use_sim_time", value=True),
            rviz_ld,
        ]),
    ])
