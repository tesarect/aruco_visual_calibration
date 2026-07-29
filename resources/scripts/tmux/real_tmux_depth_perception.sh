#!/bin/bash
# Depth-perception session, real robot: depth_perception_node — turns
# yolo_marker_bridge_node's 2D cup_holder/hole detections into stable 3D
# positions by sampling the depth image at each detection's centroid (see
# depth_perception_node.hpp). Single-node session, split out on its own
# rather than folded into real_tmux_yolo.sh since it's a genuinely separate
# concern (3D localization vs. 2D detection) with its own independent
# lifecycle.
#
# Requires BOTH the base session (real_tmux_base.sh — for move_group and
# the Zenoh-bridged D415 camera topics) AND the yolo session
# (real_tmux_yolo.sh — for yolo_marker_bridge_node, the actual publisher
# of detections_2d_topic) already up. Pane 0 polls for
# yolo_marker_bridge_node first (its real dependency — see
# depth_perception_real.yaml's detections_2d_topic), THEN move_group for
# startup-ordering consistency with every other session in this project.
#
# Per-pane logging is opt-in — pass depth_perception=on, e.g.:
#   ./real_tmux_depth_perception.sh depth_perception=on
# or turn logging on with essential_logs=on (same effect here, only one
# loggable pane):
#   ./real_tmux_depth_perception.sh essential_logs=on
# See logging.sh for what gets captured and where.

SESSION="depth_percep_real_term"
WINDOW="depth-perception"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESOURCES_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SHELL_DIR="$RESOURCES_DIR/shell"

# shellcheck source=../shell/logging.sh
source "$SHELL_DIR/logging.sh"
declare_loggable_panes depth_perception
parse_log_args "$@"
setup_log_dir "$SESSION"

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Killing existing tmux session: $SESSION"
    tmux kill-session -t "$SESSION"
fi

tmux new-session -d -s "$SESSION" -n "$WINDOW"

# Keep crashed panes visible (pane_dead=1, exit status shown) instead of
# vanishing, so node_dashboard.py can distinguish "crashed" from "never existed".
tmux set-option -t "$SESSION" remain-on-exit on

# Pane 0 — depth_perception_node. Polls for yolo_marker_bridge_node (its
# actual dependency: /aruco_perception/detections_2d has no publisher
# until that node is up — see real_tmux_yolo.sh), then move_group.
PANE0=$(tmux list-panes -t "$SESSION:$WINDOW" -F "#{pane_id}")
tmux send-keys -t "$PANE0" \
"$SHELL_DIR/wait_for_node.sh yolo_marker_bridge_node 30 && $SHELL_DIR/wait_for_node.sh move_group 30 && source ~/ros2_ws/install/setup.bash && $(wrap_log "$SESSION" depth_perception "ros2 launch depth_perception depth_perception.launch.py env:=real")" C-m

tmux select-pane -t "$PANE0" -T "Depth Perception (real)"

tmux set-option -t "$SESSION" pane-border-status top
tmux set-option -t "$SESSION" pane-border-format "#{?pane_active,#[fg=green]▶ ,}#{pane_title}"

tmux select-layout -t "$SESSION:$WINDOW" tiled

# Attach at the end
tmux select-window -t "$SESSION:$WINDOW"
tmux attach-session -t "$SESSION"