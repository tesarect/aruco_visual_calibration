#!/bin/bash
# Calibration-pipeline session: real robot equivalent of sim_tmux_trajcal.sh
# — trajectory_planner, aruco_detector_node. Runs in its own tmux session,
# independent of real_tmux_base.sh — but still requires the base session's
# nodes (Zenoh bridge, move_group, planning_scene_setup) to be up first,
# since panes here poll for them via wait_for_node.sh/wait_for_planning_scene.sh
# (ROS 2 checks, not a tmux dependency).
#
# Does NOT start calibration_broadcaster_node — calibration_broadcaster_real.yaml
# does not exist yet (see todo.txt Thread B item B4). Add a pane for it here
# once that config exists, mirroring sim_tmux_trajcal.sh's pane 2/3.

SESSION="trajcal_real_term"
WINDOW="calibration"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESOURCES_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SHELL_DIR="$RESOURCES_DIR/shell"

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Killing existing tmux session: $SESSION"
    tmux kill-session -t "$SESSION"
fi

tmux new-session -d -s "$SESSION" -n "$WINDOW"

# Pane 0 — trajectory_planner. Polls for move_group (MoveGroupInterface
# needs it to connect), THEN for the planning scene to actually contain
# 'countertop'/'wall' — trajectory_planner's move_to_home_on_startup (see
# trajectory_planner_real.yaml) plans and moves as soon as its constructor
# runs, and collision checking (on by default) can only avoid obstacles
# already in the scene at that moment. See wait_for_planning_scene.sh and
# the matching comment in sim_tmux_trajcal.sh.
PANE0=$(tmux list-panes -t "$SESSION:$WINDOW" -F "#{pane_id}")
tmux send-keys -t "$PANE0" \
"$SHELL_DIR/wait_for_node.sh move_group 30 && $SHELL_DIR/wait_for_planning_scene.sh 30 && source ~/ros2_ws/install/setup.bash && ros2 launch visual_calibration_moveit trajectory_planner.launch.py env:=real" C-m

# Pane 1 — aruco_detector_node. Needs the Zenoh bridge up (real_tmux_base.sh
# pane 0) for /D415/color/image_raw + /D415/color/camera_info to exist —
# polling for move_group here just keeps ordering consistent with the rest
# of the session's startup, same as sim_tmux_trajcal.sh.
PANE1=$(tmux split-window -t "$PANE0" -h -P -F "#{pane_id}")
tmux send-keys -t "$PANE1" \
"$SHELL_DIR/wait_for_node.sh move_group 30 && source ~/ros2_ws/install/setup.bash && ros2 run aruco_perception aruco_detector_node --ros-args --params-file ~/ros2_ws/src/visual_calibration/aruco_perception/config/aruco_detector_real.yaml" C-m

# Pane 2 — free scratch pane, ROS-sourced, for ad-hoc topic echo/debug
# (e.g. manually calling ~/trace_path to test home/cal_ready/standby moves
# before calibration_broadcaster_real.yaml exists).
PANE2=$(tmux split-window -t "$PANE1" -v -P -F "#{pane_id}")
tmux send-keys -t "$PANE2" \
"source ~/ros2_ws/install/setup.bash && source $SHELL_DIR/aliases.sh && echo 'Ready. calibration_broadcaster_node is not wired up yet (todo.txt B4) — use ~/trace_path manually for now.'" C-m

# Give each pane a title
tmux select-pane -t "$PANE0" -T "Trajectory Planner (real)"
tmux select-pane -t "$PANE1" -T "Aruco Detector (real)"
tmux select-pane -t "$PANE2" -T "Scratch"

tmux set-option -t "$SESSION" pane-border-status top
tmux set-option -t "$SESSION" pane-border-format "#{?pane_active,#[fg=green]▶ ,}#{pane_title}"

tmux select-layout -t "$SESSION:$WINDOW" tiled

# Attach at the end
tmux select-window -t "$SESSION:$WINDOW"
tmux attach-session -t "$SESSION"