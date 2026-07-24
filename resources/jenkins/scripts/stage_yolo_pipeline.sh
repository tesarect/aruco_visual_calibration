#!/bin/bash
# Jenkins "YOLO / hybrid detector (yolo pipeline)" stage.
#
# Runs aruco_perception_yolo_bridge's bringup_yolo_pipeline.launch.py, which
# starts inference_server.py (plain Flask process, ExecuteProcess — NOT a
# ROS node, see that launch file's header) gated on its own /health
# endpoint, then yolo_marker_bridge_node gated on move_group. Requires
# install_yolo.sh to have already been run on this rosject (creates
# ~/yolo_venv) — same precondition startinferenceserver/tmuxyoloreal have
# today; this stage does not install the venv itself.
#
# Independent of the orchestrator pipeline stage (parallel branch per the
# design doc's dependency chain) — Jenkins runs stages sequentially by
# default here for simplicity/clearer per-stage logs, not because there's a
# hard ordering dependency between the two.

set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./pipeline_common.sh
source "$SCRIPT_DIR/pipeline_common.sh"

RESOURCES_SHELL_DIR="$HOME/ros2_ws/src/visual_calibration/resources/scripts/shell"
ENV_NAME="${1:?Usage: stage_yolo_pipeline.sh <sim|real>}"

source ~/ros2_ws/install/setup.bash

if [ ! -x "$HOME/yolo_venv/bin/python3" ]; then
    echo "[stage_yolo_pipeline] ~/yolo_venv not found — run installyolo (install_yolo.sh) on this rosject first. Failing stage."
    exit 1
fi

echo "=== [stage_yolo_pipeline] Launching bringup_yolo_pipeline.launch.py env:=$ENV_NAME ==="
ros2 launch aruco_perception_yolo_bridge bringup_yolo_pipeline.launch.py env:="$ENV_NAME" \
    > "$LOG_DIR/yolo_pipeline_${ENV_NAME}.log" 2>&1 &
track_pid "$!" "yolo_pipeline_${ENV_NAME}"

echo "=== [stage_yolo_pipeline] Waiting for yolo_marker_bridge_node ==="
"$RESOURCES_SHELL_DIR/wait_for_node.sh" yolo_marker_bridge_node 60
if [ $? -ne 0 ]; then
    echo "[stage_yolo_pipeline] yolo_marker_bridge_node never came up — check yolo_pipeline_${ENV_NAME}.log (inference_server.py health-check and move_group waits are logged there) — failing stage."
    exit 1
fi

echo "=== [stage_yolo_pipeline] YOLO pipeline stage complete ==="
