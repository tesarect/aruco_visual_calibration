#!/bin/bash
# Calibration-pipeline session: trajectory_planner, aruco_detector_node,
# calibration_broadcaster_node. Runs in its own tmux session, independent of
# sim_tmux_base.sh — but still requires the base session's nodes (Gazebo,
# move_group) to be up first, since panes here poll for them via
# wait_for_node.sh (a ROS 2 node-list check, not a tmux dependency).

SESSION="main1_term"
# SESSION="Calib-Traj"
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
# needs it to connect).
PANE0=$(tmux list-panes -t "$SESSION:$WINDOW" -F "#{pane_id}")
tmux send-keys -t "$PANE0" \
"$SHELL_DIR/wait_for_node.sh move_group 30 && source ~/ros2_ws/install/setup.bash && ros2 launch visual_calibration_moveit trajectory_planner.launch.py env:=sim" C-m

# Pane 1 — aruco_detector_node. Only needs the camera topics (published by
# Gazebo directly, not move_group), but polling for move_group keeps the
# ordering consistent with the rest of the session's startup.
PANE1=$(tmux split-window -t "$PANE0" -h -P -F "#{pane_id}")
tmux send-keys -t "$PANE1" \
"$SHELL_DIR/wait_for_node.sh move_group 30 && source ~/ros2_ws/install/setup.bash && ros2 run aruco_perception aruco_detector_node --ros-args --params-file ~/ros2_ws/src/visual_calibration/aruco_perception/config/aruco_detector_sim.yaml" C-m

# Pane 2 — calibration_broadcaster_node. Polls for aruco_detector_node
# (needs marker_pose to be publishable) and trajectory_planner (it calls
# trajectory_planner's ~/get_polygon_waypoints + ~/trace_path itself once
# a ~/calibrate goal is sent — see pane 3).
PANE2=$(tmux split-window -t "$PANE0" -v -P -F "#{pane_id}")
tmux send-keys -t "$PANE2" \
"$SHELL_DIR/wait_for_node.sh aruco_detector_node 30 && $SHELL_DIR/wait_for_node.sh trajectory_planner 30 && source ~/ros2_ws/install/setup.bash && ros2 run aruco_perception calibration_broadcaster_node --ros-args --params-file ~/ros2_ws/src/visual_calibration/aruco_perception/config/calibration_broadcaster_sim.yaml" C-m

# Pane 3 — calibration runner: waits for calibration_broadcaster_node,
# sources aliases.sh (for startcalibration/tracepolygon), then leaves you
# at a ready prompt. Run `startcalibration` to send the ~/calibrate action
# goal — calibration_broadcaster_node drives the whole sequence itself
# (fetch waypoints, move one at a time via trajectory_planner, wait for
# settle, sample) — printing live samples_collected/total feedback and
# the final orientation spread (max/mean degrees).
PANE3=$(tmux split-window -t "$PANE1" -v -P -F "#{pane_id}")
tmux send-keys -t "$PANE3" \
"$SHELL_DIR/wait_for_node.sh calibration_broadcaster_node 30 && source ~/ros2_ws/install/setup.bash && source $SHELL_DIR/aliases.sh && echo 'Ready. Run: startcalibration'" C-m

# Give each pane a title
tmux select-pane -t "$PANE0" -T "Trajectory Planner"
tmux select-pane -t "$PANE1" -T "Aruco Detector"
tmux select-pane -t "$PANE2" -T "Calibration Broadcaster"
tmux select-pane -t "$PANE3" -T "Calibration Runner"

# Show pane titles on pane borders (must be set after the panes above
# exist — setting it earlier has no target to render into).
tmux set-option -t "$SESSION" pane-border-status top
tmux set-option -t "$SESSION" pane-border-format "#{?pane_active,#[fg=green]▶ ,}#{pane_title}"

tmux select-layout -t "$SESSION:$WINDOW" tiled

# Attach at the end
tmux select-window -t "$SESSION:$WINDOW"
tmux attach-session -t "$SESSION"
