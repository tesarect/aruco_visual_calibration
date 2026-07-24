#!/bin/bash
# Jenkins "Trajectory + Perception (orchestrator pipeline)" stage.
#
# Runs orchestrator's bringup_orchestrator_pipeline.launch.py, which
# transitively brings up (see bringup_full_sim_README.md):
#   planning_scene_setup (already run in the base stage — re-included here
#     too, since bringup_moveit_pipeline.launch.py always includes it; this
#     is harmless: it just re-populates the same collision objects) ->
#   [gate: /get_planning_scene has countertop+wall] -> trajectory_planner
#   aruco_detector_node -> [gate: node up] -> calibration_broadcaster_node
#   [gate: calibration_broadcaster_node + trajectory_planner up] ->
#   calibration_orchestrator_node
# in ONE ros2 launch call, using the SAME per-stage inline gates described
# in that README rather than re-polling from here — this stage's own job is
# just to background that one launch call, wait for it to actually reach a
# running state (calibration_orchestrator_node up), and log it to its own
# file, not to duplicate the gating logic already inside the launch files.
#
# Uses the .launch.py form, not .xml — see bringup_full_sim_README.md's
# open "XML vs Python" question; prefer Python since it's the form able to
# express additional cross-file ordering if a gap is ever found.

set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./pipeline_common.sh
source "$SCRIPT_DIR/pipeline_common.sh"

RESOURCES_SHELL_DIR="$HOME/ros2_ws/src/visual_calibration/resources/scripts/shell"
ENV_NAME="${1:?Usage: stage_orchestrator_pipeline.sh <sim|real>}"

source ~/ros2_ws/install/setup.bash

echo "=== [stage_orchestrator_pipeline] Launching bringup_orchestrator_pipeline.launch.py env:=$ENV_NAME ==="
ros2 launch orchestrator bringup_orchestrator_pipeline.launch.py env:="$ENV_NAME" \
    > "$LOG_DIR/orchestrator_pipeline_${ENV_NAME}.log" 2>&1 &
track_pid "$!" "orchestrator_pipeline_${ENV_NAME}"

echo "=== [stage_orchestrator_pipeline] Waiting for calibration_orchestrator_node ==="
"$RESOURCES_SHELL_DIR/wait_for_node.sh" calibration_orchestrator_node 120
if [ $? -ne 0 ]; then
    echo "[stage_orchestrator_pipeline] calibration_orchestrator_node never came up — check orchestrator_pipeline_${ENV_NAME}.log (this launch file's own internal gates are logged there, e.g. planning-scene/controller/node waits) — failing stage."
    exit 1
fi

echo "=== [stage_orchestrator_pipeline] Orchestrator pipeline stage complete ==="
