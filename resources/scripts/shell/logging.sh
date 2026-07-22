# Per-pane, opt-in tmux logging — sourced (not executed) by every
# */tmux/*_base.sh and */tmux/*_trajcal.sh script. Off by default (no perf/
# disk cost, no change to what a pane shows) — enable per-pane by passing
# <pane_name>=on args to the tmux script itself, e.g.:
#   ./sim_tmux_base.sh move_group=on
#   tmuxbasesim move_group=on rviz=on        # via aliases.sh, args pass through
# or enable every loggable pane in that script at once with
# essential_logs=on, e.g.:
#   ./sim_tmux_base.sh essential_logs=on
#   tmuxtrajcalreal essential_logs=on        # every wrap_log'd pane in that script
#
# What gets captured is exactly the pane's own terminal stdout/stderr (the
# same INFO/WARN/ERROR lines `ros2 launch`/`ros2 run` already print) — tee'd
# to a file as a side effect, not a second/different log source. There is
# no separate "node log" distinct from this in these scripts (they run
# nodes in the foreground, not as daemons with their own log sinks).
#
# All sessions share one flat log directory (ros2_ws/log/tmux/, no
# per-launch timestamp subfolder) — clear it yourself between sessions
# with `rm -rf ~/ros2_ws/log/tmux/*` when you want a clean slate.
#
# Usage in a *_base.sh/*_trajcal.sh script, near the top (after SESSION is
# set):
#   source "$SHELL_DIR/logging.sh"
#   declare_loggable_panes move_group rviz planning_scene   # every name
#                                                            # this script
#                                                            # passes to
#                                                            # wrap_log
#   parse_log_args "$@"
#   setup_log_dir
# then wrap each pane's command (note: SESSION now passed to wrap_log too,
# since log filenames are session-prefixed):
#   tmux send-keys -t "$PANE1" "$(wrap_log "$SESSION" move_group "ros2 launch sim_ur3e_moveit_config move_group.launch.py")" C-m
#
# declare_loggable_panes must be called BEFORE parse_log_args — it's what
# essential_logs=on expands to. If a script adds a wrap_log'd pane without
# adding its name here too, essential_logs=on simply won't cover that pane
# (same as any other silently-ignored typo in a <pane_name>=on arg) —
# individual <pane_name>=on always still works regardless.

declare -A LOG_PANES=()
declare -a LOGGABLE_PANE_NAMES=()

# Records this script's full set of wrap_log-able pane names, so
# essential_logs=on (see parse_log_args) knows what "every pane" means for
# THIS script specifically — different *_base.sh/*_trajcal.sh scripts wrap
# different pane names, there's no one global list.
declare_loggable_panes() {
    LOGGABLE_PANE_NAMES=("$@")
}

# Parses "$@" (the tmux script's own positional args) for <pane_name>=on
# tokens and records them in LOG_PANES. Unrecognized/malformed args are
# ignored (not an error) — keeps this forward-compatible with a script
# gaining other args later without this helper needing to know about them.
# essential_logs=on is shorthand for "<name>=on" for every name passed to
# declare_loggable_panes (call that first) — lets you turn on logging for
# everything this script can log without spelling out each pane every time.
parse_log_args() {
    for arg in "$@"; do
        if [ "$arg" = "essential_logs=on" ]; then
            for pane_name in "${LOGGABLE_PANE_NAMES[@]}"; do
                LOG_PANES["$pane_name"]=1
            done
        elif [[ "$arg" =~ ^([a-zA-Z0-9_]+)=on$ ]]; then
            LOG_PANES["${BASH_REMATCH[1]}"]=1
        fi
    done
}

# Ensures ros2_ws/log/tmux/ itself exists and exports LOG_DIR for wrap_log
# to use — no per-session subfolder, no timestamp: every pane's log is a
# flat file directly under here (session-prefixed, see wrap_log), and
# clearing history between sessions is a manual `rm -rf ~/ros2_ws/log/tmux/*`
# rather than something this script manages. No-op (dir not created) if no
# pane was requested via parse_log_args — avoids creating an empty
# directory on every plain, unlogged launch.
setup_log_dir() {
    if [ "${#LOG_PANES[@]}" -eq 0 ]; then
        return
    fi
    export LOG_DIR="$HOME/ros2_ws/log/tmux"
    mkdir -p "$LOG_DIR"
    echo "Logging enabled for: ${!LOG_PANES[*]}"
    echo "Log directory: $LOG_DIR"
}

# Wraps $3 (a full shell command string, as already passed to `tmux
# send-keys`) with `2>&1 | tee "$LOG_DIR/$1_$2.log"` IF $2 was requested via
# parse_log_args — otherwise returns $3 unchanged. Filename is
# <session>_<pane>.log — session-prefixed since LOG_DIR is now shared
# across every session (no per-launch subfolder), so e.g. base_term and
# trajcal_term logging the same pane name can't collide. Overwrites (plain
# tee, not tee -a) each time the wrapped command (re)starts — a restarted
# pane's log reflects only that latest run, not a mix of several restarts
# concatenated together (this was tee -a until 2026-07-22; append made it
# hard to tell which run a given log line belonged to during iterative
# rebuild/restart/retest cycles). If you want history across restarts,
# copy the file out yourself before restarting.
wrap_log() {
    local session="${1:?wrap_log requires a session name}"
    local pane_name="${2:?wrap_log requires a pane name}"
    local command="${3:?wrap_log requires a command string}"
    if [ -n "${LOG_PANES[$pane_name]:-}" ]; then
        echo "$command 2>&1 | tee \"$LOG_DIR/${session}_${pane_name}.log\""
    else
        echo "$command"
    fi
}