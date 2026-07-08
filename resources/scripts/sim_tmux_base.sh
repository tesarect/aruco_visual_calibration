#!/bin/bash
# Base tmux session: simulation, move_group, rviz, planning scene, and the
# marker-debugger (background, optional visual aid). Does not start
# trajectory_planner / aruco_detector_node / calibration_broadcaster_node —
# see sim_tmux_main1.sh for those.

SESSION="visual_calibration"
WINDOW="simulation"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Killing existing tmux session: $SESSION"
    tmux kill-session -t "$SESSION"
fi

tmux new-session -d -s "$SESSION" -n "$WINDOW"

# Pane 0 — simulation. No clean topic/service readiness signal for "Gazebo
# fully up" exists, so downstream panes gate on a fixed sleep instead of a
# poll (see wait_for_node.sh for the panes that *can* poll).
PANE0=$(tmux list-panes -t "$SESSION:$WINDOW" -F "#{pane_id}")
tmux send-keys -t "$PANE0" \
"source ~/ros2_ws/install/setup.bash && ros2 launch the_construct_office_gazebo starbots_ur3e.launch.xml" C-m

# Pane 1 — move_group. Waits a fixed 5s for Gazebo to be underway, then
# launches directly (no further poll available before it starts).
PANE1=$(tmux split-window -t "$PANE0" -h -P -F "#{pane_id}")
tmux send-keys -t "$PANE1" \
"sleep 5 && source ~/ros2_ws/install/setup.bash && ros2 launch aruco_moveit_config move_group.launch.py" C-m

# Pane 2 — rviz. Polls for move_group before launching (rviz's
# MotionPlanning plugin needs move_group up to be useful).
PANE2=$(tmux split-window -t "$PANE0" -v -P -F "#{pane_id}")
tmux send-keys -t "$PANE2" \
"$SCRIPT_DIR/wait_for_node.sh move_group 30 && source ~/ros2_ws/install/setup.bash && ros2 launch aruco_moveit_config moveit_rviz.launch.py" C-m

# Pane 3 — planning scene setup (one-shot: populates the scene, then
# exits). Polls for move_group first (PlanningSceneInterface needs it).
PANE3=$(tmux split-window -t "$PANE1" -v -P -F "#{pane_id}")
tmux send-keys -t "$PANE3" \
"$SCRIPT_DIR/wait_for_node.sh move_group 30 && source ~/ros2_ws/install/setup.bash && ros2 launch visual_calibration_moveit planning_scene_setup.launch.py env:=sim" C-m

# Pane 4 — marker-debugger (optional visual aid, RViz axis markers at key
# TF frames). Background/best-effort: polls for move_group, doesn't block
# anything else.
PANE4=$(tmux split-window -t "$PANE2" -v -P -F "#{pane_id}")
tmux send-keys -t "$PANE4" \
"$SCRIPT_DIR/wait_for_node.sh move_group 30 && python3 $SCRIPT_DIR/tf_debug_markers.py" C-m

tmux select-layout -t "$SESSION:$WINDOW" tiled

# Attach at the end
tmux select-window -t "$SESSION:$WINDOW"
tmux attach-session -t "$SESSION"
