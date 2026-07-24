#!/bin/bash
# Jenkins "Calibration run" stage — OPTIONAL, parameterized (see Jenkinsfile's
# RUN_CALIBRATION boolean param). Sends the ~/auto_calibrate action goal and
# blocks until it completes, same underlying call as aliases.sh's
# startautocalibration() — calibration_orchestrator_node drives the whole
# sequence itself (move to cal_ready, optional auto-center, then
# calibration_broadcaster_node's ~/calibrate), so this stage's only job is
# to invoke it, capture the full feedback/result stream to a log, and fail
# the stage if the action does not report success.
#
# Requires calibration_orchestrator_node already up — i.e. the orchestrator
# pipeline stage must have run first and succeeded. Not re-verified here
# (Jenkins stage ordering in the Jenkinsfile is the actual gate); this
# script just runs the action call and lets `ros2 action send_goal` itself
# fail/timeout if the node isn't there.
#
# `ros2 action send_goal ... --feedback` is a foreground, blocking call
# (not backgrounded) — no track_pid needed, it exits on its own once the
# action finishes, same as running startautocalibration by hand.

set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./pipeline_common.sh
source "$SCRIPT_DIR/pipeline_common.sh"

ENV_NAME="${1:?Usage: stage_calibration.sh <sim|real>}"
CAL_LOG="$LOG_DIR/calibration_${ENV_NAME}.log"

source ~/ros2_ws/install/setup.bash

echo "=== [stage_calibration] Sending ~/auto_calibrate goal (env=$ENV_NAME) ==="
ros2 action send_goal /calibration_orchestrator_node/auto_calibrate \
    visual_calibration_msgs/action/AutoCalibrate {} --feedback \
    > "$CAL_LOG" 2>&1
CAL_STATUS=$?

# `ros2 action send_goal`'s own exit code reflects whether the CLI call
# itself succeeded (goal sent/tracked), not necessarily the action RESULT's
# success field — grep the logged result explicitly rather than trusting
# exit code alone, since a rejected/aborted goal can still leave the CLI
# exiting 0.
if [ "$CAL_STATUS" -ne 0 ] || ! grep -q "success: true" "$CAL_LOG"; then
    echo "[stage_calibration] auto_calibrate did not report success — see calibration_${ENV_NAME}.log — failing stage."
    exit 1
fi

echo "=== [stage_calibration] Calibration run stage complete ==="
