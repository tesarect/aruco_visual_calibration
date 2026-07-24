#!/bin/bash
# Git-workflow session: two panes split vertically, one cd'd into the
# visual_calibration repo, the other into the webpage_ws repo — for running
# git commands against each repo side by side.

SESSION="GIT"
WINDOW="repos"

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Killing existing tmux session: $SESSION"
    tmux kill-session -t "$SESSION"
fi

tmux new-session -d -s "$SESSION" -n "$WINDOW"

# Keep crashed panes visible (pane_dead=1, exit status shown) instead of
# vanishing, so node_dashboard.py can distinguish "crashed" from "never existed".
tmux set-option -t "$SESSION" remain-on-exit on

# Pane 0 — visual_calibration repo
PANE0=$(tmux list-panes -t "$SESSION:$WINDOW" -F "#{pane_id}")
tmux send-keys -t "$PANE0" "cd ~/ros2_ws/src/visual_calibration" C-m

# Pane 1 — webpage_ws repo, split vertically (side-by-side)
PANE1=$(tmux split-window -t "$PANE0" -h -P -F "#{pane_id}")
tmux send-keys -t "$PANE1" "cd ~/webpage_ws" C-m

# Give each pane a title
tmux select-pane -t "$PANE0" -T "Visual cal repo"
tmux select-pane -t "$PANE1" -T "webpage repo"

# Show pane titles on pane borders (must be set after the panes above
# exist — setting it earlier has no target to render into).
tmux set-option -t "$SESSION" pane-border-status top
tmux set-option -t "$SESSION" pane-border-format "#{?pane_active,#[fg=green]▶ ,}#{pane_title}"

# Attach at the end
tmux select-window -t "$SESSION:$WINDOW"
tmux attach-session -t "$SESSION"
