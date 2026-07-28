#!/bin/bash
# Jenkins "Planning scene (sim)" stage — third/last of the four stages
# split out of the former stage_base_sim.sh. Assumes Gazebo + move_group
# are already up. Also owns the optional DEVDEBUG tools (RViz + the three
# debug_tmux.sh tools) — these depend on move_group/the running scene, not
# on anything Gazebo/move_group-stage-specific, so they belong here at the
# end of bringup rather than earlier.

set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./pipeline_common.sh
source "$SCRIPT_DIR/pipeline_common.sh"

RESOURCES_SHELL_DIR="$HOME/ros2_ws/src/visual_calibration/resources/scripts/shell"
RESOURCES_PYTHON_DIR="$HOME/ros2_ws/src/visual_calibration/resources/scripts/python"

set +u
source ~/ros2_ws/install/setup.bash
set -u

# DEVDEBUG (env var, set by the Jenkinsfile from its DEVDEBUG boolean
# parameter — default false): when true, also starts RViz (from
# sim_tmux_base.sh's own pane 2) plus the three debug_tmux.sh tools
# (tf_debug_markers.py, rqt_image_view on the ArUco overlay topic,
# rqt_graph) as backgrounded, PID-tracked processes — same commands those
# two tmux scripts use, just without tmux itself (Jenkins has no terminal
# multiplexer to attach to; the rosject's VNC/remote-desktop view is where
# these actually show up). Defaults to false so a normal/headless pipeline
# run doesn't pay the extra startup cost or clutter the desktop.
DEVDEBUG="${DEVDEBUG:-false}"

echo "=== [stage_planning_scene_sim] Running planning_scene_setup (one-shot populate) ==="
ros2 launch visual_calibration_moveit planning_scene_setup.launch.py env:=sim \
    > "$LOG_DIR/base_sim_planning_scene.log" 2>&1 &
track_pid "$!" planning_scene_setup

echo "=== [stage_planning_scene_sim] Waiting for planning scene to contain countertop+wall ==="
"$RESOURCES_SHELL_DIR/wait_for_planning_scene.sh" 30 2
if [ $? -ne 0 ]; then
    echo "[stage_planning_scene_sim] planning scene not confirmed populated within timeout — continuing (same 'continuing anyway' convention as the underlying script), but this is a strong signal to check base_sim_planning_scene.log."
fi

if [ "$DEVDEBUG" = "true" ]; then
    echo "=== [stage_planning_scene_sim] DEVDEBUG=true — starting RViz + debug tools (visible via rosject VNC) ==="

    echo "--- RViz (sim_ur3e_moveit_config) ---"
    ros2 launch sim_ur3e_moveit_config moveit_rviz.launch.py \
        > "$LOG_DIR/base_sim_rviz.log" 2>&1 &
    track_pid "$!" rviz

    echo "--- tf_debug_markers.py (polls for move_group itself) ---"
    python3 "$RESOURCES_PYTHON_DIR/tf_debug_markers.py" --env sim \
        > "$LOG_DIR/base_sim_tf_debug_markers.log" 2>&1 &
    track_pid "$!" tf_debug_markers

    echo "--- rqt_image_view (ArUco overlay topic) ---"
    ros2 run rqt_image_view rqt_image_view /aruco_perception/overlay_image \
        > "$LOG_DIR/base_sim_rqt_image_view.log" 2>&1 &
    track_pid "$!" rqt_image_view

    echo "--- rqt_graph ---"
    ros2 run rqt_graph rqt_graph \
        > "$LOG_DIR/base_sim_rqt_graph.log" 2>&1 &
    track_pid "$!" rqt_graph
fi

echo "=== [stage_planning_scene_sim] Planning scene (sim) stage complete ==="
