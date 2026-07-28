#!/bin/bash
# WINDOW variant of sim_tmux_base.sh — same panes/commands/titles/layout,
# but creates a WINDOW named "base" inside the shared "sim_deploy" SESSION
# instead of its own standalone session. Purpose-built for
# orchestrate_win_sim.sh (one session, prefix+0/1/2/3 to switch — see that
# script's header for why this exists alongside, not instead of,
# sim_tmux_base.sh). sim_tmux_base.sh itself is UNTOUCHED and still the
# right tool for standalone/independent use (own session, own lifecycle).
#
# Per-pane logging is opt-in — pass <pane_name>=on for any of simulation,
# move_group, rviz, planning_scene, e.g.:
#   ./sim_win_base.sh move_group=on
# or turn logging on for all of them at once with essential_logs=on:
#   ./sim_win_base.sh essential_logs=on
# See logging.sh for what gets captured and where.

SESSION="sim_deploy"
WINDOW="base"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESOURCES_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SHELL_DIR="$RESOURCES_DIR/shell"

# shellcheck source=../shell/logging.sh
source "$SHELL_DIR/logging.sh"
declare_loggable_panes simulation move_group rviz planning_scene
parse_log_args "$@"
setup_log_dir "$SESSION"

# Kill only THIS window if it already exists — killing the whole session
# here (like the standalone scripts do) would wipe out every OTHER
# window already running in the shared session (e.g. trajcal's window),
# which defeats the entire point of a shared multi-window session.
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

# Pane 0 — simulation.
PANE0=$(tmux list-panes -t "$SESSION:$WINDOW" -F "#{pane_id}")
tmux send-keys -t "$PANE0" \
"source ~/ros2_ws/install/setup.bash && $(wrap_log "$SESSION" simulation "ros2 launch the_construct_office_gazebo starbots_ur3e.launch.xml")" C-m

# Pane 1 — move_group. Waits for joint_state_broadcaster to be active on
# controller_manager (a real readiness signal — Gazebo's
# gazebo_ros2_control plugin has finished loading hardware AND
# controller_manager has activated it), not a fixed sleep — a flat `sleep
# N` raced against Gazebo's variable startup time and sometimes wasn't
# long enough (move_group would start while the sim controllers were
# still mid-load). See wait_for_controllers.sh.
PANE1=$(tmux split-window -t "$PANE0" -h -P -F "#{pane_id}")
tmux send-keys -t "$PANE1" \
"$SHELL_DIR/wait_for_controllers.sh /controller_manager 60 && source ~/ros2_ws/install/setup.bash && $(wrap_log "$SESSION" move_group "ros2 launch sim_ur3e_moveit_config move_group.launch.py")" C-m

# Pane 2 — rviz. Polls for move_group before launching (rviz's
# MotionPlanning plugin needs move_group up to be useful).
PANE2=$(tmux split-window -t "$PANE0" -v -P -F "#{pane_id}")
tmux send-keys -t "$PANE2" \
"$SHELL_DIR/wait_for_node.sh move_group 30 && source ~/ros2_ws/install/setup.bash && $(wrap_log "$SESSION" rviz "ros2 launch sim_ur3e_moveit_config moveit_rviz.launch.py")" C-m

# Pane 3 — planning scene setup (one-shot: populates the scene, then
# exits). Polls for move_group first (PlanningSceneInterface needs it).
PANE3=$(tmux split-window -t "$PANE1" -v -P -F "#{pane_id}")
tmux send-keys -t "$PANE3" \
"$SHELL_DIR/wait_for_node.sh move_group 30 && source ~/ros2_ws/install/setup.bash && $(wrap_log "$SESSION" planning_scene "ros2 launch visual_calibration_moveit planning_scene_setup.launch.py env:=sim")" C-m

# Pane 4 — free scratch pane, ROS-sourced, for ad-hoc topic echo/debug.
PANE4=$(tmux split-window -t "$PANE2" -v -P -F "#{pane_id}")
tmux send-keys -t "$PANE4" \
"source ~/ros2_ws/install/setup.bash && source $SHELL_DIR/aliases.sh" C-m

# Give each pane a descriptive title, matching sim_tmux_trajcal.sh's style.
tmux select-pane -t "$PANE0" -T "Simulation"
tmux select-pane -t "$PANE1" -T "MoveIt move_group"
tmux select-pane -t "$PANE2" -T "RViz"
tmux select-pane -t "$PANE3" -T "Planning Scene"
tmux select-pane -t "$PANE4" -T "Scratch"
tmux set-option -t "$SESSION" pane-border-status top
tmux set-option -t "$SESSION" pane-border-format "#{?pane_active,#[fg=green]▶ ,}#{pane_title}"

tmux select-layout -t "$SESSION:$WINDOW" tiled

# NOTE: no attach-session here, unlike sim_tmux_base.sh — this window is
# meant to be created as part of orchestrate_win_sim.sh's sequence, which
# attaches ONCE at the very end after all windows exist. Attaching here
# too would block that script from ever reaching the next window.
