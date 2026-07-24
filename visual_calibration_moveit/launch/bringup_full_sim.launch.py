"""Top-level staged bringup, sim variant, Python form. NEW, additive file —
does not replace or modify anything; sim_tmux_base.sh/sim_tmux_trajcal.sh
and all per-package launch files remain fully independent and untouched.
See bringup_full_sim_README.md (same directory) for the full design and
the dependency-chain diagram this file encodes.

Does NOT start Gazebo or move_group — those come from
the_construct_office_gazebo / sim_ur3e_moveit_config, neither a
visual_calibration/ package, so out of scope for this staged-bringup pass
(same as today: start the base session — or just Gazebo + move_group
manually — first, THEN this file). Everything from planning_scene_setup
onward is covered:

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

Compare against bringup_full_sim.xml (same directory, same sequence) — see
that file's own header for the tradeoff this pair exists to let you check
empirically (XML <include> has no cross-include readiness-waiting
mechanism; every gate here already lives inside the included files
themselves, so in principle both variants behave identically, but this is
exactly the thing to verify side by side before trusting either as a
replacement for the tmux-script chain).
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
