#!/bin/bash
# Miscellaneous terminals: 4 plain shells in a tiled layout, for ad-hoc
# work that doesn't belong to any of the other tmux sessions.

SESSION="terminals"
WINDOW="misc"

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Killing existing tmux session: $SESSION"
    tmux kill-session -t "$SESSION"
fi

tmux new-session -d -s "$SESSION" -n "$WINDOW"

PANE0=$(tmux list-panes -t "$SESSION:$WINDOW" -F "#{pane_id}")
PANE1=$(tmux split-window -t "$PANE0" -h -P -F "#{pane_id}")
PANE2=$(tmux split-window -t "$PANE0" -v -P -F "#{pane_id}")
PANE3=$(tmux split-window -t "$PANE1" -v -P -F "#{pane_id}")

tmux select-layout -t "$SESSION:$WINDOW" tiled

# Attach at the end
tmux select-window -t "$SESSION:$WINDOW"
tmux attach-session -t "$SESSION"
