#!/bin/bash
# Starts YOLO-pipeline/inference_server.py inside ~/yolo_venv, in the
# background — the always-on YOLO model server that yolo_marker_bridge_node
# calls over localhost HTTP. Not a ROS node itself (plain Flask process, no
# rclpy import ever, per the ABI-isolation rule) — see error-mitigation.md
# #15 and YOLO-pipeline/README.md.
#
# Dedicated script (not just an inline tmux command) so it can be called
# directly outside tmux too, e.g. for a manual test or a non-tmux launch
# setup. Matches this project's existing install_yolo.sh/remove_yolo.sh
# convention: a small, idempotent, standalone shell script under a
# package's own resources/scripts/, callable both directly and from a tmux
# script.
#
# Usage:
#   bash start_inference_server.sh [sim|real]
#   (env defaults to real if omitted)
#
# Backgrounds the server and returns immediately — pair with
# wait_for_inference_server.sh (same directory) to block until it's
# actually ready, same two-script pattern as wait_for_node.sh for ROS nodes.

set -euo pipefail

ENV_ARG="${1:-real}"
if [ "$ENV_ARG" != "sim" ] && [ "$ENV_ARG" != "real" ]; then
    echo " ❌ Unknown env '$ENV_ARG' — must be 'sim' or 'real'."
    exit 1
fi

YOLO_PIPELINE_DIR="$HOME/YOLO-pipeline"
VENV_DIR="$HOME/yolo_venv"
LOG_DIR="$HOME/ros2_ws/log/tmux"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/inference_server.log"

if [ ! -d "$YOLO_PIPELINE_DIR" ]; then
    echo " ❌ $YOLO_PIPELINE_DIR not found — is the YOLO-pipeline repo checked out at \$HOME?"
    exit 1
fi
if [ ! -d "$VENV_DIR" ]; then
    echo " ❌ $VENV_DIR not found — run install_yolo.sh first "
    echo "    (ros2_ws/src/visual_calibration/resources/scripts/shell/install_yolo.sh)."
    exit 1
fi

# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"
cd "$YOLO_PIPELINE_DIR"

echo " ✚ Starting inference_server.py (env=$ENV_ARG) in the background..."
echo "    Log: $LOG_FILE"
nohup python3 inference_server.py --env "$ENV_ARG" > "$LOG_FILE" 2>&1 &
SERVER_PID=$!
echo "    PID: $SERVER_PID"

deactivate

echo ""
echo "Started. Use wait_for_inference_server.sh to block until it's ready,"
echo "or check $LOG_FILE directly."