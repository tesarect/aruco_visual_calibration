#!/bin/bash
# Cup_holder/hole detector session, sim ONLY: cup_holder_detector_node
# (classical OpenCV threshold+contour, aruco_perception package) — sim's
# drop-in alternative to YOLO for cup_holder/hole detection (see
# cup_holder_detector_node.hpp's class doc comment for why sim needs its
# own detector instead of reusing sim_tmux_yolo.sh's YOLO path). Publishes
# onto the SAME /aruco_perception/detections_2d topic YOLO's bridge already
# uses on real — depth_perception_node cannot tell the difference. There is
# no real_tmux_cupholder.sh: this node has no real-robot role.
#
# Requires the base session (sim_tmux_base.sh) to be up first — polls for
# move_group. Independent of sim_tmux_trajcal.sh/sim_tmux_yolo.sh's own
# panes.
#
# Per-pane logging is opt-in — pass cup_holder_detector=on, e.g.:
#   ./sim_tmux_cupholder.sh cup_holder_detector=on
# or essential_logs=on (same thing here, single loggable pane). See
# logging.sh for what gets captured and where.

SESSION="cupholder_term"
WINDOW="cupholder"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESOURCES_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SHELL_DIR="$RESOURCES_DIR/shell"

# shellcheck source=../shell/logging.sh
source "$SHELL_DIR/logging.sh"
declare_loggable_panes cup_holder_detector
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

# Pane 0 — cup_holder_detector_node, via its env-parameterized launch file
# (env:=sim — this node has no env:=real params file, see class doc
# comment). Polls for move_group for startup-ordering consistency with
# every other sim tmux session, even though this node only needs the raw
# camera topic (published by Gazebo directly, same reasoning
# aruco_detector_node's own pane uses in sim_tmux_trajcal.sh).
PANE0=$(tmux list-panes -t "$SESSION:$WINDOW" -F "#{pane_id}")
tmux send-keys -t "$PANE0" \
"$SHELL_DIR/wait_for_node.sh move_group 30 && source ~/ros2_ws/install/setup.bash && $(wrap_log "$SESSION" cup_holder_detector "ros2 launch aruco_perception cup_holder_detector.launch.py env:=sim")" C-m

# Give the pane a title
tmux select-pane -t "$PANE0" -T "Cup Holder Detector"

tmux set-option -t "$SESSION" pane-border-status top
tmux set-option -t "$SESSION" pane-border-format "#{?pane_active,#[fg=green]▶ ,}#{pane_title}"

tmux select-layout -t "$SESSION:$WINDOW" tiled

# Attach at the end
tmux select-window -t "$SESSION:$WINDOW"
tmux attach-session -t "$SESSION"
