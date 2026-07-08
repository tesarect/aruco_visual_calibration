#!/bin/bash
# Calibration-pipeline window: trajectory_planner, aruco_detector_node,
# calibration_broadcaster_node. Assumes sim_tmux_base.sh's session (sim,
# move_group, planning scene) is already running — adds a new window to
# that same session rather than starting a fresh one. If the base session
# isn't up yet, starts it first.

SESSION="visual_calibration"
WINDOW="calibration"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if ! tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "No existing '$SESSION' session — starting sim_tmux_base.sh first."
    "$SCRIPT_DIR/sim_tmux_base.sh" &
    sleep 2
fi

if tmux list-windows -t "$SESSION" -F "#{window_name}" | grep -qx "$WINDOW"; then
    echo "Killing existing '$WINDOW' window"
    tmux kill-window -t "$SESSION:$WINDOW"
fi

tmux new-window -t "$SESSION" -n "$WINDOW"

# Pane 0 — trajectory_planner. Polls for move_group (MoveGroupInterface
# needs it to connect).
PANE0=$(tmux list-panes -t "$SESSION:$WINDOW" -F "#{pane_id}")
tmux send-keys -t "$PANE0" \
"$SCRIPT_DIR/wait_for_node.sh move_group 30 && source ~/ros2_ws/install/setup.bash && ros2 launch visual_calibration_moveit trajectory_planner.launch.py env:=sim" C-m

# Pane 1 — aruco_detector_node. Only needs the camera topics (published by
# Gazebo directly, not move_group), but polling for move_group keeps the
# ordering consistent with the rest of the session's startup.
PANE1=$(tmux split-window -t "$PANE0" -h -P -F "#{pane_id}")
tmux send-keys -t "$PANE1" \
"$SCRIPT_DIR/wait_for_node.sh move_group 30 && source ~/ros2_ws/install/setup.bash && ros2 run aruco_perception aruco_detector_node --ros-args --params-file ~/ros2_ws/src/visual_calibration/aruco_perception/config/aruco_detector_sim.yaml" C-m

# Pane 2 — calibration_broadcaster_node. Polls for aruco_detector_node
# (needs marker_pose to be publishable) and trajectory_planner (so
# ~/trace_polygon is callable once calibration starts — see pane 3).
PANE2=$(tmux split-window -t "$PANE0" -v -P -F "#{pane_id}")
tmux send-keys -t "$PANE2" \
"$SCRIPT_DIR/wait_for_node.sh aruco_detector_node 30 && $SCRIPT_DIR/wait_for_node.sh trajectory_planner 30 && source ~/ros2_ws/install/setup.bash && ros2 run aruco_perception calibration_broadcaster_node --ros-args --params-file ~/ros2_ws/src/visual_calibration/aruco_perception/config/calibration_broadcaster_sim.yaml" C-m

# Pane 3 — service-call helper: starts calibration collection, then traces
# the polygon a few times so samples spread across several arm poses (see
# CLAUDE.md/progress.md — calibration_broadcaster_node only collects, it
# never moves the arm itself). Left as a manual step for now: this pane
# just polls readiness and prints the commands rather than firing them
# automatically, since starting calibration is a deliberate user action.
PANE3=$(tmux split-window -t "$PANE1" -v -P -F "#{pane_id}")
tmux send-keys -t "$PANE3" \
"$SCRIPT_DIR/wait_for_node.sh calibration_broadcaster_node 30 && echo 'Ready. Run:' && echo '  ros2 service call /calibration_broadcaster_node/start_calibration std_srvs/srv/Trigger {}' && echo '  ros2 service call /trajectory_planner/trace_polygon std_srvs/srv/Trigger {}  # repeat until 10/10 samples collected'" C-m

tmux select-layout -t "$SESSION:$WINDOW" tiled

# Attach at the end
tmux select-window -t "$SESSION:$WINDOW"
tmux attach-session -t "$SESSION"
