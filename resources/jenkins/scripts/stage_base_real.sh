#!/bin/bash
# Jenkins "Base (real)" stage — replicates real_tmux_base.sh's Zenoh bridge +
# move_group + planning_scene_setup chain as backgrounded processes with
# readiness gates in THIS shell step. RViz intentionally NOT started
# (headless CI/demo run). Does NOT start the robot driver itself — that's
# provided by the lab environment (see CLAUDE.md), same as real_tmux_base.sh.
#
# Runs check_real_driver_fastfail.sh FIRST and fails the stage immediately
# if it doesn't pass — starting move_group/planning_scene against a driver
# that was never (re)started this session just reproduces the "move_group
# up but robot_description/tf never arrives" failure mode further
# downstream, exactly what real_tmux_base.sh's stat_check=on guards against.
# Unlike that opt-in flag, this Jenkins stage ALWAYS checks — a staged,
# watched "production" run should not silently proceed past a driver
# that isn't there.

set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./pipeline_common.sh
source "$SCRIPT_DIR/pipeline_common.sh"

RESOURCES_SHELL_DIR="$HOME/ros2_ws/src/visual_calibration/resources/scripts/shell"
source ~/ros2_ws/install/setup.bash

echo "=== [stage_base_real] Checking real robot driver status (fail-fast) ==="
if ! "$RESOURCES_SHELL_DIR/check_real_driver_fastfail.sh"; then
    echo "[stage_base_real] Real robot driver is not up — failing stage. Restart the robot driver / rosject session, then re-run."
    exit 1
fi

echo "=== [stage_base_real] Starting Zenoh bridge ==="
(cd "$HOME/ros2_ws/src/zenoh-pointcloud/init" && ./rosject.sh) \
    > "$LOG_DIR/base_real_zenoh_bridge.log" 2>&1 &
track_pid "$!" zenoh_bridge

echo "=== [stage_base_real] Ensuring scaled_joint_trajectory_controller is active ==="
"$RESOURCES_SHELL_DIR/ensure_controller_active.sh" /controller_manager scaled_joint_trajectory_controller

echo "=== [stage_base_real] Starting move_group (real_ur3e_moveit_config) ==="
ros2 launch real_ur3e_moveit_config move_group.launch.py \
    > "$LOG_DIR/base_real_move_group.log" 2>&1 &
track_pid "$!" move_group

echo "=== [stage_base_real] Waiting for move_group node ==="
"$RESOURCES_SHELL_DIR/wait_for_node.sh" move_group 60
if [ $? -ne 0 ]; then
    echo "[stage_base_real] move_group never came up — failing stage."
    exit 1
fi

echo "=== [stage_base_real] Running planning_scene_setup (one-shot populate) ==="
ros2 launch visual_calibration_moveit planning_scene_setup.launch.py env:=real \
    > "$LOG_DIR/base_real_planning_scene.log" 2>&1 &
track_pid "$!" planning_scene_setup

echo "=== [stage_base_real] Waiting for planning scene to contain countertop+wall ==="
"$RESOURCES_SHELL_DIR/wait_for_planning_scene.sh" 30 2
if [ $? -ne 0 ]; then
    echo "[stage_base_real] planning scene not confirmed populated within timeout — continuing (same 'continuing anyway' convention as the underlying script), but this is a strong signal to check base_real_planning_scene.log."
fi

echo "=== [stage_base_real] Base (real) stage complete ==="
