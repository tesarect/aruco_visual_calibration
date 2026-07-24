#!/bin/bash
# Jenkins "Base (sim)" stage — replicates sim_tmux_base.sh's Gazebo +
# move_group + planning_scene_setup chain as backgrounded processes with
# readiness gates in THIS shell step (not a separate stage assuming
# readiness — see error-mitigation.md #17/#36 and the design doc's warning
# about racing a `sh` step that backgrounds something and returns
# immediately). RViz is intentionally NOT started here — headless CI/demo
# run, no display needed; add back if the user wants it visible during a
# presentation.
#
# Does NOT use sim_tmux_base.sh itself (that's the tmux/dev path, untouched
# by this work) — replicates its commands directly per the design doc,
# since bringup_full_sim's chain explicitly excludes Gazebo/move_group.

set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./pipeline_common.sh
source "$SCRIPT_DIR/pipeline_common.sh"

RESOURCES_SHELL_DIR="$HOME/ros2_ws/src/visual_calibration/resources/scripts/shell"
source ~/ros2_ws/install/setup.bash

echo "=== [stage_base_sim] Starting Gazebo (starbots_ur3e.launch.xml) ==="
ros2 launch the_construct_office_gazebo starbots_ur3e.launch.xml \
    > "$LOG_DIR/base_sim_gazebo.log" 2>&1 &
track_pid "$!" gazebo

echo "=== [stage_base_sim] Waiting for joint_state_broadcaster active ==="
"$RESOURCES_SHELL_DIR/wait_for_controllers.sh" /controller_manager 90
CONTROLLER_STATUS=$?
if [ "$CONTROLLER_STATUS" -ne 0 ]; then
    echo "[stage_base_sim] controller_manager never reported joint_state_broadcaster active — failing stage."
    exit 1
fi

echo "=== [stage_base_sim] Starting move_group (sim_ur3e_moveit_config) ==="
ros2 launch sim_ur3e_moveit_config move_group.launch.py \
    > "$LOG_DIR/base_sim_move_group.log" 2>&1 &
track_pid "$!" move_group

echo "=== [stage_base_sim] Waiting for move_group node ==="
"$RESOURCES_SHELL_DIR/wait_for_node.sh" move_group 60
if [ $? -ne 0 ]; then
    echo "[stage_base_sim] move_group never came up — failing stage."
    exit 1
fi

echo "=== [stage_base_sim] Running planning_scene_setup (one-shot populate) ==="
ros2 launch visual_calibration_moveit planning_scene_setup.launch.py env:=sim \
    > "$LOG_DIR/base_sim_planning_scene.log" 2>&1 &
track_pid "$!" planning_scene_setup

echo "=== [stage_base_sim] Waiting for planning scene to contain countertop+wall ==="
"$RESOURCES_SHELL_DIR/wait_for_planning_scene.sh" 30 2
if [ $? -ne 0 ]; then
    echo "[stage_base_sim] planning scene not confirmed populated within timeout — continuing (same 'continuing anyway' convention as the underlying script), but this is a strong signal to check base_sim_planning_scene.log."
fi

echo "=== [stage_base_sim] Base (sim) stage complete ==="
