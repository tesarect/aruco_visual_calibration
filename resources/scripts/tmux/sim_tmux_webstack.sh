#!/bin/bash
# Web dashboard session: rosbridge websocket, plus npm install +
# build/preview/proxy for the React app under webpage_ws/app — the latter
# is all delegated to webpage_ws/setup_rosject.sh followed by
# `npm run start` (build, then background preview + the reverse proxy —
# see app/package.json), so this script stays a thin tmux wrapper, not a
# second place that duplicates that sequence. Independent of
# sim_tmux_base.sh/sim_tmux_trajcal.sh.
#
# Jenkins is NOT touched by this script — it has its own standalone
# lifecycle (see the `startjenkins` alias / install_jenkins.sh), started
# independently of whether this web session is ever run. This session is
# web-dashboard-only.
#
# Usage: ./sim_tmux_webstack.sh [sim|real] [pane_name=on ...]
# Picks which env's URDF the dashboard pane targets (see setup_rosject.sh's
# header — URDF extraction is now a one-time, committed-file thing, not
# re-derived from a live robot every session; each env is tracked
# separately, one does NOT also cover the other).
# Per-pane logging is opt-in — pass <pane_name>=on for rosbridge (vite/
# proxy are not wrap_log'd — they background themselves inside the
# dashboard pane's own chain, so a foreground `tee` there wouldn't usefully
# capture much beyond the synchronous build step anyway), e.g.
# ./sim_tmux_webstack.sh sim rosbridge=on, or essential_logs=on (same
# effect here, only one loggable pane) — see logging.sh for what gets
# captured and where.

ENV="${1:-sim}"
if [ "$ENV" != "sim" ] && [ "$ENV" != "real" ]; then
    echo "Usage: $0 [sim|real] [pane_name=on ...]" >&2
    exit 1
fi
shift || true

SESSION="webstack_term"
WINDOW="web-interface"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESOURCES_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SHELL_DIR="$RESOURCES_DIR/shell"

# shellcheck source=../shell/logging.sh
source "$SHELL_DIR/logging.sh"
declare_loggable_panes rosbridge
parse_log_args "$@"
setup_log_dir

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Killing existing tmux session: $SESSION"
    tmux kill-session -t "$SESSION"
fi

tmux new-session -d -s "$SESSION" -n "$WINDOW"

# Keep crashed panes visible (pane_dead=1, exit status shown) instead of
# vanishing, so node_dashboard.py can distinguish "crashed" from "never existed".
tmux set-option -t "$SESSION" remain-on-exit on

# Pane 0 (top) — starts `rosbridge webserver socket`
PANE0=$(tmux list-panes -t "$SESSION:$WINDOW" -F "#{pane_id}")
tmux send-keys -t "$PANE0" \
"$(wrap_log "$SESSION" rosbridge "ros2 launch rosbridge_server rosbridge_websocket_launch.xml")" C-m

# Pane 1 (bottom, split from pane 0, horizontal) — setup_rosject.sh for the
# env passed in ($ENV, default sim: node/nvm, one-time URDF extraction only
# if not already committed, npm install — covers a wiped node_modules or a
# newly-added dependency like http-proxy), THEN `npm run start` (build,
# then background preview + proxy — see app/package.json; NOT the old
# PORT=7000 pattern, which predates the proxy and no longer binds
# anything). setup_rosject.sh also kills stale proxy/preview processes up
# front (see scripts/stop_stale.sh), so this is safe to re-run after a
# rosject restart mid-session, not just on a pristine one. All strictly
# sequential, one pane/one `&&` chain. Invoked via `bash`, not
# `./setup_rosject.sh` — the file's executable bit isn't set (plain file
# copies drop it, same gotcha CLAUDE.md notes for scripts under
# resources/scripts/) and `bash` sidesteps needing it.
PANE1=$(tmux split-window -t "$PANE0" -v -P -F "#{pane_id}")
tmux send-keys -t "$PANE1" \
"cd ~/webpage_ws && bash ./setup_rosject.sh --env $ENV && source ~/webpage_ws/scripts/session_init.sh && cd ~/webpage_ws/app && npm run start" C-m

# Give each pane a title
tmux select-pane -t "$PANE0" -T "ROS Bridge WebSocket"
tmux select-pane -t "$PANE1" -T "Dashboard + Proxy"

# Show pane titles on pane borders (must be set after the panes above
# exist — setting it earlier has no target to render into).
tmux set-option -t "$SESSION" pane-border-status top
tmux set-option -t "$SESSION" pane-border-format "#{?pane_active,#[fg=green]▶ ,}#{pane_title}"

tmux select-layout -t "$SESSION:$WINDOW" tiled

# Attach at the end
tmux select-window -t "$SESSION:$WINDOW"
tmux attach-session -t "$SESSION"
