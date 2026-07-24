"""Top-level staged bringup, real-robot variant, Python form. NEW, additive
file — does not replace or modify anything; real_tmux_base.sh/
real_tmux_trajcal.sh and all per-package launch files remain fully
independent and untouched. See bringup_full_real_README.md (same
directory) for the full design.

Identical structure to bringup_full_sim.launch.py (same file, see its
header for the full dependency-chain diagram) — the only difference is
env:=real is threaded through to every included pipeline, since every
per-package bringup_*.launch.py already parameterizes on env. Does NOT
start the Zenoh bridge, controller-activation (ensure_controller_active.sh),
or move_group — none of those are visual_calibration/ packages, same
out-of-scope reasoning as Gazebo/move_group in the sim variant. Start the
real base session (or at minimum: Zenoh bridge, ensure_controller_active.sh,
move_group) first, THEN this file — same ordering real_tmux_base.sh already
requires before real_tmux_trajcal.sh today.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    orchestrator_pipeline_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("orchestrator"),
                "launch", "bringup_orchestrator_pipeline.launch.py",
            ])
        ),
        launch_arguments={"env": "real"}.items(),
    )

    yolo_pipeline_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("aruco_perception_yolo_bridge"),
                "launch", "bringup_yolo_pipeline.launch.py",
            ])
        ),
        launch_arguments={"env": "real"}.items(),
    )

    return LaunchDescription([
        orchestrator_pipeline_include,
        yolo_pipeline_include,
    ])
