"""Top-level staged bringup for simulation: orchestrator pipeline and YOLO
perception pipeline from a single `ros2 launch` call.

Does not start Gazebo or move_group — those come from
the_construct_office_gazebo / sim_ur3e_moveit_config, neither a
visual_calibration/ package. Start those first, then this file. Full
dependency chain encoded by the includes below:

  bringup_orchestrator_pipeline.launch.py (this include)
    -> bringup_moveit_pipeline.launch.py
         -> planning_scene_setup.launch.py
         -> [gate: /get_planning_scene has {"countertop","wall"}]
         -> trajectory_planner.launch.py
    -> bringup_aruco_pipeline.launch.py
         -> aruco_detector.launch.py
         -> [gate: aruco_detector_node up]
         -> calibration_broadcaster.launch.py
    -> [gate: calibration_broadcaster_node + trajectory_planner up]
    -> calibration_orchestrator.launch.py

  bringup_yolo_pipeline.launch.py (this include, parallel branch)
    -> inference_server.py (ExecuteProcess, not a ROS node)
    -> [gate: inference_server.py /health up]
    -> [gate: move_group up]
    -> yolo_marker_bridge.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    env_arg = DeclareLaunchArgument(
        "env",
        default_value="sim",
        description="Which parameter file to load: 'sim' or 'real'",
    )

    orchestrator_pipeline_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("orchestrator"),
                "launch", "bringup_orchestrator_pipeline.launch.py",
            ])
        ),
        launch_arguments={"env": LaunchConfiguration("env")}.items(),
    )

    yolo_pipeline_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("aruco_perception_yolo_bridge"),
                "launch", "bringup_yolo_pipeline.launch.py",
            ])
        ),
        launch_arguments={"env": LaunchConfiguration("env")}.items(),
    )

    return LaunchDescription([
        env_arg,
        orchestrator_pipeline_include,
        yolo_pipeline_include,
    ])
