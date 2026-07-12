#!/bin/bash
# Calibration-pipeline session: trajectory_planner, aruco_detector_node,
# calibration_broadcaster_node. Runs in its own tmux session, independent of
# sim_tmux_base.sh — but still requires the base session's nodes (Gazebo,
# move_group) to be up first, since panes here poll for them via
# wait_for_node.sh (a ROS 2 node-list check, not a tmux dependency).

SESSION="main3_term"
# SESSION="Web-Stack"
WINDOW="web-interface"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESOURCES_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SHELL_DIR="$RESOURCES_DIR/shell"

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Killing existing tmux session: $SESSION"
    tmux kill-session -t "$SESSION"
fi

tmux new-session -d -s "$SESSION" -n "$WINDOW"

# Pane 0 — starts `rosbridge webserver socket`
PANE0=$(tmux list-panes -t "$SESSION:$WINDOW" -F "#{pane_id}")
tmux send-keys -t "$PANE0" \
"ros2 launch rosbridge_server rosbridge_websocket_launch.xml" C-m

# Pane 1 — vite fast dev server call
PANE1=$(tmux split-window -t "$PANE0" -h -P -F "#{pane_id}")
tmux send-keys -t "$PANE1" \
"source ~/webpage_ws/scripts/session_init.sh && cd ~/webpage_ws/app && PORT=7000 npm run build dev && PORT=7000 npm run preview" C-m


# Give each pane a title
tmux select-pane -t "$PANE0" -T "ROS Bridge WebSocket"
tmux select-pane -t "$PANE1" -T "Vite Run"
# tmux select-pane -t "$PANE2" -T "Calibration Broadcaster"
# tmux select-pane -t "$PANE3" -T "Calibration Runner"

# Show pane titles on pane borders (must be set after the panes above
# exist — setting it earlier has no target to render into).
tmux set-option -t "$SESSION" pane-border-status top
tmux set-option -t "$SESSION" pane-border-format "#{?pane_active,#[fg=green]▶ ,}#{pane_title}"

tmux select-layout -t "$SESSION:$WINDOW" tiled

# Attach at the end
tmux select-window -t "$SESSION:$WINDOW"
tmux attach-session -t "$SESSION"
