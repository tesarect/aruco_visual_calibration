#!/bin/bash
# Debug tmux session: tf_debug_markers.py (RViz axis markers at key TF
# frames), rqt_image_view loaded directly on the ArUco overlay topic,
# rqt_graph, and a large empty shell at ~/ros2_ws for ad-hoc work. Pulled
# out of sim_tmux_base.sh/real_tmux_base.sh so this always-optional visual
# debugging isn't tied to the base session's lifecycle.
#
# Requires the matching base session (sim_tmux_base.sh/real_tmux_base.sh)
# already up — tf_debug_markers.py polls for move_group itself.
#
# Usage: ./debug_tmux.sh [sim|real]   (default: sim)
# Picks tf_debug_markers.py's --env (frame names differ: sim uses the RG2
# gripper's rg2_gripper_* frames, real uses Robotiq 85's robotiq_85_*
# frames — see tf_debug_markers.py's header for why).

ENV="${1:-sim}"
if [ "$ENV" != "sim" ] && [ "$ENV" != "real" ]; then
    echo "Usage: $0 [sim|real]" >&2
    exit 1
fi

SESSION="debug_term"
WINDOW="debug"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESOURCES_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SHELL_DIR="$RESOURCES_DIR/shell"
PYTHON_DIR="$RESOURCES_DIR/python"

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Killing existing tmux session: $SESSION"
    tmux kill-session -t "$SESSION"
fi

tmux new-session -d -s "$SESSION" -n "$WINDOW"

# Keep crashed panes visible (pane_dead=1, exit status shown) instead of
# vanishing, so node_dashboard.py can distinguish "crashed" from "never existed".
tmux set-option -t "$SESSION" remain-on-exit on

# Pane 0 — large shell at ~/ros2_ws, created first so main-vertical below
# treats it as the main/large pane. Left empty on purpose (scratch).
PANE0=$(tmux list-panes -t "$SESSION:$WINDOW" -F "#{pane_id}")
tmux send-keys -t "$PANE0" \
"source ~/ros2_ws/install/setup.bash && source $SHELL_DIR/aliases.sh && cd ~/ros2_ws" C-m

# Pane 1 — tf_debug_markers.py. Polls for move_group (see sim_tmux_base.sh/
# real_tmux_base.sh — this session doesn't start move_group itself).
PANE1=$(tmux split-window -t "$PANE0" -h -P -F "#{pane_id}")
tmux send-keys -t "$PANE1" \
"$SHELL_DIR/wait_for_node.sh move_group 30 && python3 $PYTHON_DIR/tf_debug_markers.py --env $ENV" C-m

# Pane 2 — rqt_image_view, loaded directly on the ArUco overlay topic
# (same as the viewoverlaycam alias — see aliases.sh).
PANE2=$(tmux split-window -t "$PANE1" -v -P -F "#{pane_id}")
tmux send-keys -t "$PANE2" \
"ros2 run rqt_image_view rqt_image_view /aruco_perception/overlay_image" C-m

# Pane 3 — rqt_graph.
PANE3=$(tmux split-window -t "$PANE1" -v -P -F "#{pane_id}")
tmux send-keys -t "$PANE3" \
"ros2 run rqt_graph rqt_graph" C-m

tmux select-pane -t "$PANE0" -T "Scratch (ros2_ws)"
tmux select-pane -t "$PANE1" -T "Marker Debugger"
tmux select-pane -t "$PANE2" -T "Overlay Image"
tmux select-pane -t "$PANE3" -T "RQt Graph"

tmux set-option -t "$SESSION" pane-border-status top
tmux set-option -t "$SESSION" pane-border-format "#{?pane_active,#[fg=green]▶ ,}#{pane_title}"

# main-vertical: pane 0 (the shell) stays large on the left; the other
# three stack narrow on the right, per main-pane-width below.
tmux set-option -t "$SESSION" main-pane-width "65%"
tmux select-layout -t "$SESSION:$WINDOW" main-vertical

# Attach at the end
tmux select-window -t "$SESSION:$WINDOW"
tmux attach-session -t "$SESSION"
