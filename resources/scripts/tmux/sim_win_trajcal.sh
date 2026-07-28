#!/bin/bash
# WINDOW variant of sim_tmux_trajcal.sh — see sim_win_base.sh's header for
# why this parallel set of scripts exists (one shared "sim_deploy" SESSION,
# each script adds its own WINDOW instead of its own session). Same
# panes/commands/titles/layout as sim_tmux_trajcal.sh; that script is
# UNTOUCHED and still the right tool for standalone use.
#
# Per-pane logging is opt-in — pass <pane_name>=on for any of
# trajectory_planner, aruco_detector, calibration_broadcaster,
# calibration_orchestrator, e.g.:
#   ./sim_win_trajcal.sh trajectory_planner=on
# or turn logging on for all of them at once with essential_logs=on:
#   ./sim_win_trajcal.sh essential_logs=on
# See logging.sh for what gets captured and where.

SESSION="sim_deploy"
WINDOW="trajcal"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESOURCES_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SHELL_DIR="$RESOURCES_DIR/shell"

# shellcheck source=../shell/logging.sh
source "$SHELL_DIR/logging.sh"
declare_loggable_panes trajectory_planner aruco_detector calibration_broadcaster calibration_orchestrator
parse_log_args "$@"
setup_log_dir "$SESSION"

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

# Pane 0 — trajectory_planner. Polls for move_group (MoveGroupInterface
# needs it to connect), THEN for the planning scene to actually contain
# 'countertop'/'wall' (not just move_group being reachable) — trajectory_
# planner's move_to_home_on_startup (see trajectory_planner_sim.yaml) plans
# and moves as soon as its constructor runs, and collision checking can
# only avoid obstacles already in the scene at that moment. See
# wait_for_planning_scene.sh.
PANE0=$(tmux list-panes -t "$SESSION:$WINDOW" -F "#{pane_id}")
tmux send-keys -t "$PANE0" \
"$SHELL_DIR/wait_for_node.sh move_group 30 && $SHELL_DIR/wait_for_planning_scene.sh 30 && source ~/ros2_ws/install/setup.bash && $(wrap_log "$SESSION" trajectory_planner "ros2 launch visual_calibration_moveit trajectory_planner.launch.py env:=sim")" C-m

# Pane 1 — aruco_detector_node. Only needs the camera topics (published by
# Gazebo directly, not move_group), but polling for move_group keeps the
# ordering consistent with the rest of the session's startup.
PANE1=$(tmux split-window -t "$PANE0" -h -P -F "#{pane_id}")
tmux send-keys -t "$PANE1" \
"$SHELL_DIR/wait_for_node.sh move_group 30 && source ~/ros2_ws/install/setup.bash && $(wrap_log "$SESSION" aruco_detector "ros2 run aruco_perception aruco_detector_node --ros-args --params-file ~/ros2_ws/src/visual_calibration/aruco_perception/config/aruco_detector_sim.yaml")" C-m

# Pane 2 — calibration_broadcaster_node. Polls for aruco_detector_node
# (needs marker_pose to be publishable) and trajectory_planner (it calls
# trajectory_planner's ~/get_polygon_waypoints + ~/trace_path itself once
# a ~/calibrate goal is sent — see pane 3).
PANE2=$(tmux split-window -t "$PANE0" -v -P -F "#{pane_id}")
tmux send-keys -t "$PANE2" \
"$SHELL_DIR/wait_for_node.sh aruco_detector_node 30 && $SHELL_DIR/wait_for_node.sh trajectory_planner 30 && source ~/ros2_ws/install/setup.bash && $(wrap_log "$SESSION" calibration_broadcaster "ros2 run aruco_perception calibration_broadcaster_node --ros-args --params-file ~/ros2_ws/src/visual_calibration/aruco_perception/config/calibration_broadcaster_sim.yaml")" C-m

# Pane 3 — calibration_orchestrator_node. Polls for calibration_broadcaster_
# node (it calls its ~/calibrate action as a client) and trajectory_planner
# (it calls ~/get_standoff_pose + ~/trace_path itself for the cal_ready move
# and, if auto_center_enabled, the auto-center probe — see
# calibration_orchestrator_sim.yaml). Exposes ~/auto_calibrate, which chains
# cal_ready -> optional auto-center -> ~/calibrate into one action goal.
PANE3=$(tmux split-window -t "$PANE0" -v -P -F "#{pane_id}")
tmux send-keys -t "$PANE3" \
"$SHELL_DIR/wait_for_node.sh calibration_broadcaster_node 30 && $SHELL_DIR/wait_for_node.sh trajectory_planner 30 && source ~/ros2_ws/install/setup.bash && $(wrap_log "$SESSION" calibration_orchestrator "ros2 launch orchestrator calibration_orchestrator.launch.py env:=sim")" C-m

# Give each pane a title
tmux select-pane -t "$PANE0" -T "Trajectory Planner"
tmux select-pane -t "$PANE1" -T "Aruco Detector"
tmux select-pane -t "$PANE2" -T "Calibration Broadcaster"
tmux select-pane -t "$PANE3" -T "Calibration Orchestrator"

tmux set-option -t "$SESSION" pane-border-status top
tmux set-option -t "$SESSION" pane-border-format "#{?pane_active,#[fg=green]▶ ,}#{pane_title}"

tmux select-layout -t "$SESSION:$WINDOW" tiled

# NOTE: no attach-session here — see sim_win_base.sh's matching comment.
