#!/bin/bash
# Thin wrapper around wait_for_planning_scene.py (resources/scripts/python/)
# — see that file for the full explanation and why this is Python, not a
# `ros2 service call` bash loop (an earlier bash version calling
# `ros2 service call` in a tight ~1s loop is believed to have crashed
# move_group once during testing, via repeated client connect/disconnect
# churn against /get_planning_scene).
#
# Usage: wait_for_planning_scene.sh [timeout_sec] [interval_sec]
source ~/ros2_ws/install/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_DIR="$(cd "$SCRIPT_DIR/../python" && pwd)"

TIMEOUT_SEC="${1:-30}"
INTERVAL_SEC="${2:-2}"

python3 "$PYTHON_DIR/wait_for_planning_scene.py" --timeout "$TIMEOUT_SEC" --interval "$INTERVAL_SEC"