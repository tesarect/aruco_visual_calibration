#!/bin/bash
# Master launcher, real robot: real_win_base.sh, real_win_trajcal.sh,
# real_win_yolo.sh (all with essential_logs=on) and win_debug.sh real (no
# logs) in order, as four WINDOWS in one shared "real_deploy" tmux
# session, then attaches once. See orchestrate_tmux_sim.sh's header for
# the full sequencing/attach rationale — identical here, just the real_
# variants. Same BASE_DELAY_SEC fixed head start for base before
# trajcal+yolo (both counted from base's start, not chained); debug fires
# immediately after with no delay.
BASE_DELAY_SEC=60
#
# Does NOT start the robot driver itself (UR driver / ros2_control /
# robot_state_publisher) — same as real_win_base.sh alone, that's
# provided by the lab environment outside this project (see CLAUDE.md).
# Run realrobotstatuscheck first if unsure the driver is up (or pass
# stat_check=on-style gating manually via real_win_base.sh directly).
#
# Usage: ./orchestrate_tmux_real.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSION="real_deploy"

echo "=== [orchestrate_tmux_real] Starting base window (Zenoh bridge/move_group/rviz/planning_scene) ==="
bash "$SCRIPT_DIR/real_win_base.sh" essential_logs=on

echo "=== [orchestrate_tmux_real] Waiting ${BASE_DELAY_SEC}s before trajcal/yolo (head start for base) ==="
sleep "$BASE_DELAY_SEC"

echo "=== [orchestrate_tmux_real] Starting trajcal window (trajectory + calibration pipeline) ==="
bash "$SCRIPT_DIR/real_win_trajcal.sh" essential_logs=on

echo "=== [orchestrate_tmux_real] Starting yolo window (YOLO/hybrid-detector) ==="
bash "$SCRIPT_DIR/real_win_yolo.sh" essential_logs=on

echo "=== [orchestrate_tmux_real] Starting debug window (no logs) ==="
bash "$SCRIPT_DIR/win_debug.sh" real

echo "=== [orchestrate_tmux_real] All windows launched in session '$SESSION': base, trajcal, yolo, debug ==="
echo "=== [orchestrate_tmux_real] Attaching — switch windows with prefix+0/1/2/3 or prefix+w ==="

tmux attach-session -t "$SESSION"
