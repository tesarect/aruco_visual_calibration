#!/bin/bash
# Real robot equivalent of sim_tmux_webstack.sh — rosbridge websocket +
# vite web app. Identical commands to the sim version (webpage_ws has no
# real/sim split), just its own session so it doesn't collide with a sim
# run and shows up separately in node_dashboard.py.

SESSION="webstack_real_term"
WINDOW="web-interface"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESOURCES_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SHELL_DIR="$RESOURCES_DIR/shell"

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Killing existing tmux session: $SESSION"
    tmux kill-session -t "$SESSION"
fi

tmux new-session -d -s "$SESSION" -n "$WINDOW"

# Keep crashed panes visible (pane_dead=1, exit status shown) instead of
# vanishing, so node_dashboard.py can distinguish "crashed" from "never existed".
tmux set-option -t "$SESSION" remain-on-exit on

# Pane 0 — starts `rosbridge webserver socket`
PANE0=$(tmux list-panes -t "$SESSION:$WINDOW" -F "#{pane_id}")
tmux send-keys -t "$PANE0" \
"ros2 launch rosbridge_server rosbridge_websocket_launch.xml" C-m

# Pane 1 — vite fast dev server call
PANE1=$(tmux split-window -t "$PANE0" -h -P -F "#{pane_id}")
tmux send-keys -t "$PANE1" \
"source ~/webpage_ws/scripts/session_init.sh && cd ~/webpage_ws/app && PORT=7000 npm run build && PORT=7000 npm run preview" C-m


# Give each pane a title
tmux select-pane -t "$PANE0" -T "ROS Bridge WebSocket"
tmux select-pane -t "$PANE1" -T "Vite Run"

# Show pane titles on pane borders (must be set after the panes above
# exist — setting it earlier has no target to render into).
tmux set-option -t "$SESSION" pane-border-status top
tmux set-option -t "$SESSION" pane-border-format "#{?pane_active,#[fg=green]▶ ,}#{pane_title}"

tmux select-layout -t "$SESSION:$WINDOW" tiled

# Attach at the end
tmux select-window -t "$SESSION:$WINDOW"
tmux attach-session -t "$SESSION"
