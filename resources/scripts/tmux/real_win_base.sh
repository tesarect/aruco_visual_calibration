#!/bin/bash
# WINDOW variant of real_tmux_base.sh — see sim_win_base.sh's header for why
# this parallel set of scripts exists (one shared "real_deploy" SESSION,
# each script adds its own WINDOW instead of its own session). Same
# panes/commands/titles/layout/stat_check gate as real_tmux_base.sh; that
# script is UNTOUCHED and still the right tool for standalone use.
#
# Driver status check is opt-in — pass stat_check=on to run it before
# touching tmux, e.g. `./real_win_base.sh stat_check=on`.
#
# Per-pane logging is opt-in — pass <pane_name>=on for any of zenoh_bridge,
# move_group, rviz, planning_scene, or essential_logs=on for all of them.
# See logging.sh for what gets captured and where.

SESSION="real_deploy"
WINDOW="base"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESOURCES_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SHELL_DIR="$RESOURCES_DIR/shell"

# shellcheck source=../shell/logging.sh
source "$SHELL_DIR/logging.sh"
declare_loggable_panes zenoh_bridge move_group rviz planning_scene
parse_log_args "$@"
setup_log_dir "$SESSION"

STAT_CHECK=false
for arg in "$@"; do
    [ "$arg" = "stat_check=on" ] && STAT_CHECK=true
done

if [ "$STAT_CHECK" = true ]; then
    echo "Checking real robot driver status before starting $SESSION:$WINDOW..."
    if ! "$SHELL_DIR/check_real_driver_fastfail.sh"; then
        echo
        echo "Some drivers are not up yet, so disconnect & reconnect (restart the"
        echo "real robot driver / rosject session), then re-run this script."
        exit 1
    fi
    echo
fi

# Kill only THIS window if it exists — see sim_win_base.sh's comment on why
# (killing the whole session would wipe out every other window in it).
if tmux has-session -t "$SESSION" 2>/dev/null; then
    if tmux list-windows -t "$SESSION" -F "#{window_name}" | grep -qx "$WINDOW"; then
        echo "Killing existing window: $SESSION:$WINDOW"
        tmux kill-window -t "$SESSION:$WINDOW"
    fi
    tmux new-window -t "$SESSION" -n "$WINDOW"
else
    tmux new-session -d -s "$SESSION" -n "$WINDOW"
fi

# Keep crashed panes visible (pane_dead=1, exit status shown) instead of
# vanishing, so node_dashboard.py can distinguish "crashed" from "never existed".
tmux set-option -t "$SESSION" remain-on-exit on

# Pane 0 — Zenoh bridge (zenoh-bridge-ros2dds).
PANE0=$(tmux list-panes -t "$SESSION:$WINDOW" -F "#{pane_id}")
tmux send-keys -t "$PANE0" \
"cd ~/ros2_ws/src/zenoh-pointcloud/init && $(wrap_log "$SESSION" zenoh_bridge "./rosject.sh")" C-m

# Pane 1 — move_group (real_ur3e_moveit_config). Runs
# ensure_controller_active.sh first — scaled_joint_trajectory_controller
# has been observed dropping to inactive intermittently on real.
PANE1=$(tmux split-window -t "$PANE0" -h -P -F "#{pane_id}")
tmux send-keys -t "$PANE1" \
"source ~/ros2_ws/install/setup.bash && $SHELL_DIR/ensure_controller_active.sh /controller_manager scaled_joint_trajectory_controller; $(wrap_log "$SESSION" move_group "ros2 launch real_ur3e_moveit_config move_group.launch.py")" C-m

# Pane 2 — rviz. Polls for move_group before launching.
PANE2=$(tmux split-window -t "$PANE1" -h -P -F "#{pane_id}")
tmux send-keys -t "$PANE2" \
"$SHELL_DIR/wait_for_node.sh move_group 30 && source ~/ros2_ws/install/setup.bash && $(wrap_log "$SESSION" rviz "ros2 launch real_ur3e_moveit_config moveit_rviz.launch.py")" C-m

# Pane 3 — planning scene setup (one-shot). Polls for move_group first.
PANE3=$(tmux split-window -t "$PANE1" -v -P -F "#{pane_id}")
tmux send-keys -t "$PANE3" \
"$SHELL_DIR/wait_for_node.sh move_group 30 && source ~/ros2_ws/install/setup.bash && $(wrap_log "$SESSION" planning_scene "ros2 launch visual_calibration_moveit planning_scene_setup.launch.py env:=real")" C-m

# Pane 4 — free scratch pane, ROS-sourced, for ad-hoc topic echo/debug.
PANE4=$(tmux split-window -t "$PANE2" -v -P -F "#{pane_id}")
tmux send-keys -t "$PANE4" \
"source ~/ros2_ws/install/setup.bash && source $SHELL_DIR/aliases.sh" C-m

tmux select-pane -t "$PANE0" -T "Zenoh Bridge"
tmux select-pane -t "$PANE1" -T "MoveIt move_group (real)"
tmux select-pane -t "$PANE2" -T "RViz"
tmux select-pane -t "$PANE3" -T "Planning Scene"
tmux select-pane -t "$PANE4" -T "Scratch"
tmux set-option -t "$SESSION" pane-border-status top
tmux set-option -t "$SESSION" pane-border-format "#{?pane_active,#[fg=green]▶ ,}#{pane_title}"

tmux select-layout -t "$SESSION:$WINDOW" tiled

# NOTE: no attach-session here — see sim_win_base.sh's matching comment.
