#!/bin/bash
# WINDOW variant of real_tmux_yolo.sh — see sim_win_base.sh's header for why
# this parallel set of scripts exists (one shared "real_deploy" SESSION,
# each script adds its own WINDOW instead of its own session). Same
# panes/commands/titles/layout as real_tmux_yolo.sh; that script is
# UNTOUCHED and still the right tool for standalone use.
#
# Per-pane logging is opt-in — pass <pane_name>=on for inference_server or
# yolo_marker_bridge, or essential_logs=on for both.
# See logging.sh for what gets captured and where.

SESSION="real_deploy"
WINDOW="yolo"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESOURCES_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SHELL_DIR="$RESOURCES_DIR/shell"
# See real_tmux_yolo.sh's comment on why this needs TWO levels up, not one.
YOLO_BRIDGE_SCRIPTS_DIR="$RESOURCES_DIR/../../aruco_perception_yolo_bridge/resources/scripts/shell"

# shellcheck source=../shell/logging.sh
source "$SHELL_DIR/logging.sh"
declare_loggable_panes inference_server yolo_marker_bridge
parse_log_args "$@"
setup_log_dir "$SESSION"

# Kill only THIS window if it exists — see sim_win_base.sh's comment on why
# (killing the whole session would wipe out every other window in it).
if tmux has-session -t "$SESSION" 2>/dev/null; then
    if tmux list-windows -t "$SESSION" -F "#{window_name}" | grep -qx "$WINDOW"; then
        echo "Killing existing window: $SESSION:$WINDOW"
        tmux kill-window -t "$SESSION:$WINDOW"
    fi
    tmux new-window -t "$SESSION" -n "$WINDOW"
else
    tmux new-session -d -s "$SESSION" -n "$WINDOW"
fi

# Keep crashed panes visible (pane_dead=1, exit status shown) instead of
# vanishing, so node_dashboard.py can distinguish "crashed" from "never existed".
tmux set-option -t "$SESSION" remain-on-exit on

# Pane 0 — inference_server.py. NOT a ROS node.
PANE0=$(tmux list-panes -t "$SESSION:$WINDOW" -F "#{pane_id}")
tmux send-keys -t "$PANE0" \
"$(wrap_log "$SESSION" inference_server "bash $YOLO_BRIDGE_SCRIPTS_DIR/start_inference_server.sh real && $YOLO_BRIDGE_SCRIPTS_DIR/wait_for_inference_server.sh 30 real")" C-m

# Pane 1 — yolo_marker_bridge_node. Polls for inference_server.py's
# readiness, then move_group.
PANE1=$(tmux split-window -t "$PANE0" -h -P -F "#{pane_id}")
tmux send-keys -t "$PANE1" \
"$YOLO_BRIDGE_SCRIPTS_DIR/wait_for_inference_server.sh 30 real && $SHELL_DIR/wait_for_node.sh move_group 30 && source ~/ros2_ws/install/setup.bash && $(wrap_log "$SESSION" yolo_marker_bridge "ros2 run aruco_perception_yolo_bridge yolo_marker_bridge_node.py --ros-args --params-file ~/ros2_ws/src/visual_calibration/aruco_perception_yolo_bridge/config/yolo_marker_bridge_real.yaml")" C-m

# Give each pane a title
tmux select-pane -t "$PANE0" -T "Inference Server (YOLO, real)"
tmux select-pane -t "$PANE1" -T "Yolo Marker Bridge (real)"

tmux set-option -t "$SESSION" pane-border-status top
tmux set-option -t "$SESSION" pane-border-format "#{?pane_active,#[fg=green]▶ ,}#{pane_title}"

tmux select-layout -t "$SESSION:$WINDOW" tiled

# NOTE: no attach-session here — see sim_win_base.sh's matching comment.
