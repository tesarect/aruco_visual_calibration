"""Top-level staged bringup for the real robot: orchestrator pipeline
(planning scene, trajectory planner, ArUco detection, calibration
broadcast, calibration orchestrator) plus the YOLO perception pipeline,
all with env fixed to "real".

Does not start the Zenoh bridge, controller activation, or move_group —
none of those are visual_calibration/ packages. Those must already be
running before this launch file, mirroring bringup_full_sim.launch.py's
dependency-chain structure (see that file's header) with env:=real threaded
through every included pipeline.
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
