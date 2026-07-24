#!/bin/bash
# YOLO/hybrid-detector session, real robot: inference_server.py +
# yolo_marker_bridge_node. Split out of real_tmux_trajcal.sh (2026-07-23)
# to keep that session down to its core calibration-pipeline panes —
# these two panes are independent of it otherwise, only sharing the
# requirement that move_group (from real_tmux_base.sh) is already up.
#
# Requires the base session (real_tmux_base.sh) to be up first — pane 1
# polls for move_group. Independent of real_tmux_trajcal.sh's own panes
# (does not poll for trajectory_planner/aruco_detector_node/
# calibration_broadcaster_node), but calibration_orchestrator_node (in
# trajcal) is what actually calls ~/set_detector_mode to switch into
# hybrid mode — start both sessions if you intend to test that switch.
#
# Per-pane logging is opt-in — pass <pane_name>=on for inference_server or
# yolo_marker_bridge, e.g.:
#   ./real_tmux_yolo.sh inference_server=on yolo_marker_bridge=on
# or turn logging on for both at once with essential_logs=on:
#   ./real_tmux_yolo.sh essential_logs=on
# See logging.sh for what gets captured and where.
# inference_server (pane 0) — always-on YOLO model server, not a ROS node
#   (see start_inference_server.sh) — needed for the classical/hybrid
#   switch (calibration_orchestrator_node's ~/set_detector_mode) even
#   though aruco_detector_node (classical) is still the default active
#   detector on startup. start_inference_server.sh backgrounds the actual
#   server and returns, then wait_for_inference_server.sh blocks this
#   pane until it's ready.
# yolo_marker_bridge (pane 1) — the hybrid-mode detector, a real ROS
#   node, subscribed and running continuously but only publishing
#   marker_pose once switched active (see yolo_marker_bridge_node.py's
#   class doc comment).

SESSION="yolo_real_term"
WINDOW="yolo"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESOURCES_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SHELL_DIR="$RESOURCES_DIR/shell"
# RESOURCES_DIR is visual_calibration/resources/scripts — the YOLO bridge
# package is a SIBLING of visual_calibration/resources (not resources/
# itself), so this needs to go up TWO levels (past scripts/, past
# resources/) to reach visual_calibration/, then down into
# aruco_perception_yolo_bridge/. Fixed 2026-07-23 — the previous
# RESOURCES_DIR/../aruco_perception_yolo_bridge/... (only one level up)
# resolved to resources/aruco_perception_yolo_bridge/..., which doesn't
# exist; this was a pre-existing bug carried over from the original
# real_tmux_trajcal.sh's identical (also broken) computation, never
# noticed until testing YOLO from a real tmux session actually hit it.
YOLO_BRIDGE_SCRIPTS_DIR="$RESOURCES_DIR/../../aruco_perception_yolo_bridge/resources/scripts/shell"

# shellcheck source=../shell/logging.sh
source "$SHELL_DIR/logging.sh"
declare_loggable_panes inference_server yolo_marker_bridge
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

# Pane 0 — inference_server.py. NOT a ROS node (plain Flask process inside
# ~/yolo_venv, never imports rclpy/cv_bridge — see error-mitigation.md #15),
# so it has no ROS dependency to poll for and starts as soon as the pane
# exists.
PANE0=$(tmux list-panes -t "$SESSION:$WINDOW" -F "#{pane_id}")
tmux send-keys -t "$PANE0" \
"$(wrap_log "$SESSION" inference_server "bash $YOLO_BRIDGE_SCRIPTS_DIR/start_inference_server.sh real && $YOLO_BRIDGE_SCRIPTS_DIR/wait_for_inference_server.sh 30 real")" C-m

# Pane 1 — yolo_marker_bridge_node. A real ROS node — polls for
# inference_server.py's own readiness first (via wait_for_inference_server.sh,
# not wait_for_node.sh, since that server isn't a ROS node), THEN move_group
# for startup-ordering consistency (matches real_tmux_base.sh being started
# first).
PANE1=$(tmux split-window -t "$PANE0" -h -P -F "#{pane_id}")
tmux send-keys -t "$PANE1" \
"$YOLO_BRIDGE_SCRIPTS_DIR/wait_for_inference_server.sh 30 real && $SHELL_DIR/wait_for_node.sh move_group 30 && source ~/ros2_ws/install/setup.bash && $(wrap_log "$SESSION" yolo_marker_bridge "ros2 run aruco_perception_yolo_bridge yolo_marker_bridge_node.py --ros-args --params-file ~/ros2_ws/src/visual_calibration/aruco_perception_yolo_bridge/config/yolo_marker_bridge_real.yaml")" C-m

# Give each pane a title
tmux select-pane -t "$PANE0" -T "Inference Server (YOLO, real)"
tmux select-pane -t "$PANE1" -T "Yolo Marker Bridge (real)"

tmux set-option -t "$SESSION" pane-border-status top
tmux set-option -t "$SESSION" pane-border-format "#{?pane_active,#[fg=green]▶ ,}#{pane_title}"

tmux select-layout -t "$SESSION:$WINDOW" tiled

# Attach at the end
tmux select-window -t "$SESSION:$WINDOW"
tmux attach-session -t "$SESSION"
