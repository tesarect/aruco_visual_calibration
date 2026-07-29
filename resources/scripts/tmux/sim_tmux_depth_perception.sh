#!/bin/bash
# Depth-perception session, sim: cup_holder_detector_node (top pane) +
# depth_perception_node (bottom pane). Two panes, one session — merged
# 2026-07-29 (previously cup_holder_detector_node lived in its own
# sim_tmux_cupholder.sh, now removed) since depth_perception_node has no
# real purpose in sim without cup_holder_detector_node already feeding it,
# and vice versa there's little reason to run the classical detector alone
# outside dataset/tuning work (use sim_tmux_percep.sh for that instead).
#
# cup_holder_detector_node (aruco_perception package, classical OpenCV
# threshold+contour) is sim's drop-in alternative to
# yolo_marker_bridge_node for cup_holder/hole detection — sim's CPU-only
# rosject can't run the YOLO inference server fast enough for usable
# detection (see cup_holder_detector_node.hpp's class doc comment).
# Publishes onto the SAME /aruco_perception/detections_2d topic YOLO's
# bridge uses on real — depth_perception_node cannot tell the difference.
# There is no real-robot equivalent of this node; real still uses
# yolo_marker_bridge_node directly — see real_tmux_depth_perception.sh.
#
# Requires the base session (sim_tmux_base.sh) up first — both panes poll
# for move_group. Independent of sim_tmux_trajcal.sh/sim_tmux_yolo.sh's
# own panes.
#
# Per-pane logging is opt-in — pass cup_holder_detector=on and/or
# depth_perception=on, e.g.:
#   ./sim_tmux_depth_perception.sh cup_holder_detector=on depth_perception=on
# or turn logging on for both at once with essential_logs=on:
#   ./sim_tmux_depth_perception.sh essential_logs=on
# See logging.sh for what gets captured and where.

SESSION="depth_percep_term"
WINDOW="depth-perception"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESOURCES_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SHELL_DIR="$RESOURCES_DIR/shell"

# shellcheck source=../shell/logging.sh
source "$SHELL_DIR/logging.sh"
declare_loggable_panes cup_holder_detector depth_perception
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

# Pane 0 (top) — cup_holder_detector_node, via its env-parameterized
# launch file (env:=sim — this node has no env:=real params file, see
# class doc comment). Polls for move_group for startup-ordering
# consistency with every other sim tmux session, even though this node
# only needs the raw camera topic (published by Gazebo directly, same
# reasoning aruco_detector_node's own pane uses in sim_tmux_trajcal.sh).
PANE0=$(tmux list-panes -t "$SESSION:$WINDOW" -F "#{pane_id}")
tmux send-keys -t "$PANE0" \
"$SHELL_DIR/wait_for_node.sh move_group 30 && source ~/ros2_ws/install/setup.bash && $(wrap_log "$SESSION" cup_holder_detector "ros2 launch aruco_perception cup_holder_detector.launch.py env:=sim")" C-m

# Pane 1 (bottom, split from pane 0, horizontal) — depth_perception_node.
# Polls for cup_holder_detector_node (its real dependency: this pane's own
# /aruco_perception/detections_2d has no publisher until pane 0 is up —
# see depth_perception_sim.yaml's detections_2d_topic), THEN move_group.
PANE1=$(tmux split-window -t "$PANE0" -v -P -F "#{pane_id}")
tmux send-keys -t "$PANE1" \
"$SHELL_DIR/wait_for_node.sh cup_holder_detector_node 30 && $SHELL_DIR/wait_for_node.sh move_group 30 && source ~/ros2_ws/install/setup.bash && $(wrap_log "$SESSION" depth_perception "ros2 launch depth_perception depth_perception.launch.py env:=sim")" C-m

tmux select-pane -t "$PANE0" -T "Cup Holder Detector"
tmux select-pane -t "$PANE1" -T "Depth Perception"

tmux set-option -t "$SESSION" pane-border-status top
tmux set-option -t "$SESSION" pane-border-format "#{?pane_active,#[fg=green]▶ ,}#{pane_title}"

tmux select-layout -t "$SESSION:$WINDOW" tiled

# Attach at the end
tmux select-window -t "$SESSION:$WINDOW"
tmux attach-session -t "$SESSION"
