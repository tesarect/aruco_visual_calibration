#!/bin/bash
# Calibration-pipeline session: real robot equivalent of sim_tmux_trajcal.sh
# — trajectory_planner, aruco_detector_node, calibration_broadcaster_node,
# calibration_orchestrator_node. Runs in its own tmux session, independent
# of real_tmux_base.sh — but still requires the base session's nodes (Zenoh
# bridge, move_group, planning_scene_setup) to be up first, since panes
# here poll for them via wait_for_node.sh/wait_for_planning_scene.sh (ROS 2
# checks, not a tmux dependency).
#
# calibration_broadcaster_real.yaml now exists (todo.txt B4, closed
# 2026-07-18) — this session is no longer trajectory_planner/aruco_detector_
# node only. NONE of this has been tested live against the real robot yet
# (see todo.txt) — expect to iterate on calibration_broadcaster_real.yaml's
# placeholder values (num_samples, sample_wait_timeout_sec) once it has.

# Per-pane logging is opt-in — pass <pane_name>=on for any of
# trajectory_planner, aruco_detector, calibration_broadcaster,
# calibration_orchestrator, inference_server, yolo_marker_bridge, e.g.:
#   ./real_tmux_trajcal.sh trajectory_planner=on
# or turn logging on for all of them at once with essential_logs=on:
#   ./real_tmux_trajcal.sh essential_logs=on
# See logging.sh for what gets captured and where.
# trajectory_planner (pane 0)
# aruco_detector (pane 1)
# calibration_broadcaster (pane 2)
# calibration_orchestrator (pane 3)
# inference_server (pane 4) — always-on YOLO model server, not a ROS node
#   (see start_inference_server.sh) — needed for the classical/hybrid
#   switch (calibration_orchestrator_node's ~/set_detector_mode) even
#   though aruco_detector_node (classical) is still the default active
#   detector on startup. Was the "calibration runner" ready-prompt pane;
#   that prompt ran no persistent process of its own (just echoed a
#   reminder and sat idle) and is dropped here — startautocalibration/
#   startcalibration are still reachable from any regular terminal after
#   sourcing aliases.sh, same as always.
# yolo_marker_bridge (pane 5) — the hybrid-mode detector, a real ROS node,
#   subscribed and running continuously but only publishing marker_pose
#   once switched active (see yolo_marker_bridge_node.py's class doc
#   comment).

SESSION="trajcal_real_term"
WINDOW="calibration"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESOURCES_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SHELL_DIR="$RESOURCES_DIR/shell"
YOLO_BRIDGE_SCRIPTS_DIR="$RESOURCES_DIR/../aruco_perception_yolo_bridge/resources/scripts/shell"

# shellcheck source=../shell/logging.sh
source "$SHELL_DIR/logging.sh"
declare_loggable_panes trajectory_planner aruco_detector calibration_broadcaster calibration_orchestrator inference_server yolo_marker_bridge
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

# Pane 0 — trajectory_planner. Polls for move_group (MoveGroupInterface
# needs it to connect), THEN for the planning scene to actually contain
# 'countertop'/'wall' — trajectory_planner's move_to_home_on_startup (see
# trajectory_planner_real.yaml) plans and moves as soon as its constructor
# runs, and collision checking (on by default) can only avoid obstacles
# already in the scene at that moment. See wait_for_planning_scene.sh and
# the matching comment in sim_tmux_trajcal.sh.
PANE0=$(tmux list-panes -t "$SESSION:$WINDOW" -F "#{pane_id}")
tmux send-keys -t "$PANE0" \
"$SHELL_DIR/wait_for_node.sh move_group 30 && $SHELL_DIR/wait_for_planning_scene.sh 30 && source ~/ros2_ws/install/setup.bash && $(wrap_log "$SESSION" trajectory_planner "ros2 launch visual_calibration_moveit trajectory_planner.launch.py env:=real")" C-m

# Pane 1 — aruco_detector_node. Needs the Zenoh bridge up (real_tmux_base.sh
# pane 0) for /D415/color/image_raw + /D415/color/camera_info to exist —
# polling for move_group here just keeps ordering consistent with the rest
# of the session's startup, same as sim_tmux_trajcal.sh.
PANE1=$(tmux split-window -t "$PANE0" -h -P -F "#{pane_id}")
tmux send-keys -t "$PANE1" \
"$SHELL_DIR/wait_for_node.sh move_group 30 && source ~/ros2_ws/install/setup.bash && $(wrap_log "$SESSION" aruco_detector "ros2 run aruco_perception aruco_detector_node --ros-args --params-file ~/ros2_ws/src/visual_calibration/aruco_perception/config/aruco_detector_real.yaml")" C-m

# Pane 2 — calibration_broadcaster_node. Polls for aruco_detector_node
# (needs marker_pose to be publishable) and trajectory_planner (it calls
# trajectory_planner's ~/get_polygon_waypoints + ~/trace_path itself once a
# ~/calibrate goal is sent). See calibration_broadcaster_real.yaml's header
# comment for real's inverted known_chain_frame/marker_frame mounting
# (camera fixed, marker rides the arm — opposite of sim).
PANE2=$(tmux split-window -t "$PANE0" -v -P -F "#{pane_id}")
tmux send-keys -t "$PANE2" \
"$SHELL_DIR/wait_for_node.sh aruco_detector_node 30 && $SHELL_DIR/wait_for_node.sh trajectory_planner 30 && source ~/ros2_ws/install/setup.bash && $(wrap_log "$SESSION" calibration_broadcaster "ros2 run aruco_perception calibration_broadcaster_node --ros-args --params-file ~/ros2_ws/src/visual_calibration/aruco_perception/config/calibration_broadcaster_real.yaml")" C-m

# Pane 3 — calibration_orchestrator_node. Polls for calibration_broadcaster_
# node and trajectory_planner. Exposes ~/auto_calibrate (cal_ready move ->
# auto-center, ON by default on real, see calibration_orchestrator_real.yaml
# -> ~/calibrate). This is what the web app's Calibrate button should call
# once wired (todo.txt roswebdev note pending confirmation this works live).
PANE3=$(tmux split-window -t "$PANE1" -v -P -F "#{pane_id}")
tmux send-keys -t "$PANE3" \
"$SHELL_DIR/wait_for_node.sh calibration_broadcaster_node 30 && $SHELL_DIR/wait_for_node.sh trajectory_planner 30 && source ~/ros2_ws/install/setup.bash && $(wrap_log "$SESSION" calibration_orchestrator "ros2 launch orchestrator calibration_orchestrator.launch.py env:=real")" C-m

# Pane 4 — inference_server.py. NOT a ROS node (plain Flask process inside
# ~/yolo_venv, never imports rclpy/cv_bridge — see error-mitigation.md #15),
# so it has no ROS dependency to poll for and starts as soon as the pane
# exists. Always started regardless of which detector is currently active
# (aruco_detector_node/classical is the default) — kept "pre-heated" so
# switching to hybrid via ~/set_detector_mode never pays a cold model-load
# cost. start_inference_server.sh backgrounds the actual server and returns,
# then wait_for_inference_server.sh blocks this pane until it's ready.
PANE4=$(tmux split-window -t "$PANE2" -v -P -F "#{pane_id}")
tmux send-keys -t "$PANE4" \
"$(wrap_log "$SESSION" inference_server "bash $YOLO_BRIDGE_SCRIPTS_DIR/start_inference_server.sh real && $YOLO_BRIDGE_SCRIPTS_DIR/wait_for_inference_server.sh 30 real")" C-m

# Pane 5 — yolo_marker_bridge_node. A real ROS node — polls for
# inference_server.py's own readiness first (via wait_for_inference_server.sh,
# not wait_for_node.sh, since that server isn't a ROS node), THEN move_group
# for startup-ordering consistency with the rest of this session, same as
# aruco_detector's pane 1. Subscribed and running continuously from
# startup (detecting cup_holder/hole every frame regardless of the
# classical/hybrid switch, see class doc comment) but its marker_pose
# publish stays gated off (active: false, see yolo_marker_bridge_real.yaml)
# until calibration_orchestrator_node's ~/set_detector_mode flips it.
PANE5=$(tmux split-window -t "$PANE3" -v -P -F "#{pane_id}")
tmux send-keys -t "$PANE5" \
"$YOLO_BRIDGE_SCRIPTS_DIR/wait_for_inference_server.sh 30 real && $SHELL_DIR/wait_for_node.sh move_group 30 && source ~/ros2_ws/install/setup.bash && $(wrap_log "$SESSION" yolo_marker_bridge "ros2 run aruco_perception_yolo_bridge yolo_marker_bridge_node.py --ros-args --params-file ~/ros2_ws/src/visual_calibration/aruco_perception_yolo_bridge/config/yolo_marker_bridge_real.yaml")" C-m

# Give each pane a title
tmux select-pane -t "$PANE0" -T "Trajectory Planner (real)"
tmux select-pane -t "$PANE1" -T "Aruco Detector (real)"
tmux select-pane -t "$PANE2" -T "Calibration Broadcaster (real)"
tmux select-pane -t "$PANE3" -T "Calibration Orchestrator (real)"
tmux select-pane -t "$PANE4" -T "Inference Server (YOLO, real)"
tmux select-pane -t "$PANE5" -T "Yolo Marker Bridge (real)"

tmux set-option -t "$SESSION" pane-border-status top
tmux set-option -t "$SESSION" pane-border-format "#{?pane_active,#[fg=green]▶ ,}#{pane_title}"

tmux select-layout -t "$SESSION:$WINDOW" tiled

# Attach at the end
tmux select-window -t "$SESSION:$WINDOW"
tmux attach-session -t "$SESSION"