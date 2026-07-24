#!/bin/bash
# Base tmux session: real robot equivalent of sim_tmux_base.sh — Zenoh
# bridge, move_group, rviz, planning scene. Does NOT start the robot driver
# itself (UR driver / ros2_control / robot_state_publisher) — that's
# provided by the lab environment, outside this project's launch files (see
# CLAUDE.md's Real Robot Camera Setup). The marker-debugger
# (tf_debug_markers.py) lives in its own debug_tmux.sh session now, not here.
#
# Driver status check is opt-in — pass stat_check=on to run it before
# touching tmux, e.g. `./real_tmux_base.sh stat_check=on`. Uses
# check_real_driver_fastfail.sh (checks controller_manager, /joint_states,
# /tf IN ORDER, exits immediately on the first failure — no need to wait
# for a full report just to get a yes/no here). check_real_driver.sh
# itself is untouched and still the tool for a full mid-session report
# (see aliases.sh's realrobotstatuscheck). Without stat_check=on, this
# script does NOT check driver status at all and goes straight to tmux.

# Per-pane logging is opt-in — pass <pane_name>=on for any of zenoh_bridge,
# move_group, rviz, planning_scene, e.g.:
#   ./real_tmux_base.sh move_group=on
# or turn logging on for all of them at once with essential_logs=on:
#   ./real_tmux_base.sh essential_logs=on
# See logging.sh for what gets captured and where.

SESSION="base_real_term"
WINDOW="real"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESOURCES_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SHELL_DIR="$RESOURCES_DIR/shell"

# shellcheck source=../shell/logging.sh
source "$SHELL_DIR/logging.sh"
declare_loggable_panes zenoh_bridge move_group rviz planning_scene
parse_log_args "$@"
setup_log_dir

STAT_CHECK=false
for arg in "$@"; do
    [ "$arg" = "stat_check=on" ] && STAT_CHECK=true
done

if [ "$STAT_CHECK" = true ]; then
    # Gate the whole session on the real driver actually being up —
    # starting move_group/rviz/etc. against a driver that never
    # (re)started this session just reproduces the "move_group up but
    # robot_description/tf never arrives" failure mode further
    # downstream, where it's harder to diagnose.
    echo "Checking real robot driver status before starting $SESSION..."
    if ! "$SHELL_DIR/check_real_driver_fastfail.sh"; then
        echo
        echo "Some drivers are not up yet, so disconnect & reconnect (restart the"
        echo "real robot driver / rosject session), then re-run this script."
        exit 1
    fi
    echo
fi

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Killing existing tmux session: $SESSION"
    tmux kill-session -t "$SESSION"
fi

tmux new-session -d -s "$SESSION" -n "$WINDOW"

# Keep crashed panes visible (pane_dead=1, exit status shown) instead of
# vanishing, so node_dashboard.py can distinguish "crashed" from "never existed".
tmux set-option -t "$SESSION" remain-on-exit on

# Pane 0 — Zenoh bridge (zenoh-bridge-ros2dds). Long-running/foreground —
# bridges the real camera's DDS topics over Zenoh (bandwidth constraints,
# see CLAUDE.md's Real Robot Camera Setup). Required for /D415/* topics to
# reach this session at all — aruco_detector_node/yolo_marker_bridge_node
# (see real_tmux_trajcal.sh) have nothing to subscribe to without this.
# Requires install_zenoh.sh to have already run once (see setup_real.sh).
PANE0=$(tmux list-panes -t "$SESSION:$WINDOW" -F "#{pane_id}")
tmux send-keys -t "$PANE0" \
"cd ~/ros2_ws/src/zenoh-pointcloud/init && $(wrap_log "$SESSION" zenoh_bridge "./rosject.sh")" C-m

# Pane 1 — move_group. Uses real_ur3e_moveit_config (project-owned copy
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
PANE1=$(tmux split-window -t "$PANE0" -h -P -F "#{pane_id}")
tmux send-keys -t "$PANE1" \
"source ~/ros2_ws/install/setup.bash && $SHELL_DIR/ensure_controller_active.sh /controller_manager scaled_joint_trajectory_controller; $(wrap_log "$SESSION" move_group "ros2 launch real_ur3e_moveit_config move_group.launch.py")" C-m

# Pane 2 — rviz. Polls for move_group before launching.
PANE2=$(tmux split-window -t "$PANE1" -h -P -F "#{pane_id}")
tmux send-keys -t "$PANE2" \
"$SHELL_DIR/wait_for_node.sh move_group 30 && source ~/ros2_ws/install/setup.bash && $(wrap_log "$SESSION" rviz "ros2 launch real_ur3e_moveit_config moveit_rviz.launch.py")" C-m

# Pane 3 — planning scene setup (one-shot: populates the scene, then
# exits). Polls for move_group first (PlanningSceneInterface needs it).
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

# Attach at the end
tmux select-window -t "$SESSION:$WINDOW"
tmux attach-session -t "$SESSION"