#!/bin/bash
# Base tmux session: real robot equivalent of sim_tmux_base.sh — move_group,
# rviz, planning scene, marker-debugger. Does NOT start the robot driver
# itself (UR driver / ros2_control / robot_state_publisher) — that's
# provided by the lab environment, outside this project's launch files (see
# CLAUDE.md's Real Robot Camera Setup and check_real_driver.sh). Run
# `realrobotstatuscheck` first and confirm /joint_states + /tf are
# publishing before starting this session.

SESSION="base_real_term"
WINDOW="real"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESOURCES_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SHELL_DIR="$RESOURCES_DIR/shell"
PYTHON_DIR="$RESOURCES_DIR/python"

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Killing existing tmux session: $SESSION"
    tmux kill-session -t "$SESSION"
fi

tmux new-session -d -s "$SESSION" -n "$WINDOW"

# Pane 0 — move_group. Uses real_ur3e_moveit_config (project-owned copy
# under visual_calibration/, mirroring sim_ur3e_moveit_config's pattern)
# — NOT universal_robot_ros2/ur3e_moveit_config directly, since that
# instructor-provided package's moveit_controllers.yaml was found to have
# drifted to sim's controller name (joint_trajectory_controller, no
# scaled_ prefix) at some point — real_ur3e_moveit_config has the correct
# scaled_joint_trajectory_controller. Assumes the robot driver is already
# up — check first with `realrobotstatuscheck`. Runs
# ensure_controller_active.sh first: scaled_joint_trajectory_controller
# has been observed dropping to inactive intermittently on real (root
# cause not yet diagnosed) — this activates it if needed before
# move_group starts relying on it.
PANE0=$(tmux list-panes -t "$SESSION:$WINDOW" -F "#{pane_id}")
tmux send-keys -t "$PANE0" \
"source ~/ros2_ws/install/setup.bash && $SHELL_DIR/ensure_controller_active.sh /controller_manager scaled_joint_trajectory_controller; ros2 launch real_ur3e_moveit_config move_group.launch.py" C-m

# Pane 1 — rviz. Polls for move_group before launching.
PANE1=$(tmux split-window -t "$PANE0" -h -P -F "#{pane_id}")
tmux send-keys -t "$PANE1" \
"$SHELL_DIR/wait_for_node.sh move_group 30 && source ~/ros2_ws/install/setup.bash && ros2 launch real_ur3e_moveit_config moveit_rviz.launch.py" C-m

# Pane 2 — planning scene setup (one-shot: populates the scene, then
# exits). Polls for move_group first (PlanningSceneInterface needs it).
PANE2=$(tmux split-window -t "$PANE0" -v -P -F "#{pane_id}")
tmux send-keys -t "$PANE2" \
"$SHELL_DIR/wait_for_node.sh move_group 30 && source ~/ros2_ws/install/setup.bash && ros2 launch visual_calibration_moveit planning_scene_setup.launch.py env:=real" C-m

# Pane 3 — marker-debugger (optional visual aid, RViz axis markers at key
# TF frames). Background/best-effort: polls for move_group.
PANE3=$(tmux split-window -t "$PANE1" -v -P -F "#{pane_id}")
tmux send-keys -t "$PANE3" \
"$SHELL_DIR/wait_for_node.sh move_group 30 && python3 $PYTHON_DIR/tf_debug_markers.py" C-m

# Pane 4 — free scratch pane, ROS-sourced, for ad-hoc topic echo/debug.
PANE4=$(tmux split-window -t "$PANE3" -v -P -F "#{pane_id}")
tmux send-keys -t "$PANE4" \
"source ~/ros2_ws/install/setup.bash && source $SHELL_DIR/aliases.sh" C-m

tmux select-pane -t "$PANE0" -T "MoveIt move_group (real)"
tmux select-pane -t "$PANE1" -T "RViz"
tmux select-pane -t "$PANE2" -T "Planning Scene"
tmux select-pane -t "$PANE3" -T "Marker Debugger"
tmux select-pane -t "$PANE4" -T "Scratch"
tmux set-option -t "$SESSION" pane-border-status top
tmux set-option -t "$SESSION" pane-border-format "#{?pane_active,#[fg=green]▶ ,}#{pane_title}"

tmux select-layout -t "$SESSION:$WINDOW" tiled

# Attach at the end
tmux select-window -t "$SESSION:$WINDOW"
tmux attach-session -t "$SESSION"