#!/bin/bash
# Base tmux session: simulation, move_group, rviz, planning scene. Does not
# start trajectory_planner / aruco_detector_node / calibration_broadcaster_node
# — see sim_tmux_main1.sh for those. The marker-debugger (tf_debug_markers.py)
# lives in its own debug_tmux.sh session now, not here.
#
# Per-pane logging is opt-in — pass <pane_name>=on for any of simulation,
# move_group, rviz, planning_scene, e.g.:
#   ./sim_tmux_base.sh move_group=on
# or turn logging on for all of them at once with essential_logs=on:
#   ./sim_tmux_base.sh essential_logs=on
# See logging.sh for what gets captured and where.

SESSION="base_term"
WINDOW="simulation"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESOURCES_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SHELL_DIR="$RESOURCES_DIR/shell"

# shellcheck source=../shell/logging.sh
source "$SHELL_DIR/logging.sh"
declare_loggable_panes simulation move_group rviz planning_scene
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

# Attach at the end
tmux select-window -t "$SESSION:$WINDOW"
tmux attach-session -t "$SESSION"
