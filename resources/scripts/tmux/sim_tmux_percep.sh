#!/bin/bash
# Perception-experiments session: camera viewing/capture + an isolated
# YOLO venv pane, for Task 3 target-detection work on the Barista's top
# plate. Runs in its own tmux session, independent of sim_tmux_base.sh —
# but still requires the base session's nodes (Gazebo) to be up for the
# camera topics to exist, since panes here poll for them via
# wait_for_node.sh (a ROS 2 node-list check, not a tmux dependency).
#
# YOLO itself is NOT auto-started here — it lives in its own venv
# (~/yolo_venv, see install_yolo.sh) kept deliberately separate from any
# pane that sources ROS's setup.bash, to avoid the cv_bridge/OpenCV ABI
# conflict risk (error-mitigation.md #15). Pane 2 just activates the venv
# and waits at a ready prompt.

SESSION="percep_term"
WINDOW="Image-Perception"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESOURCES_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SHELL_DIR="$RESOURCES_DIR/shell"
PYTHON_DIR="$RESOURCES_DIR/python"

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Killing existing tmux session: $SESSION"
    tmux kill-session -t "$SESSION"
fi

tmux new-session -d -s "$SESSION" -n "$WINDOW"

# Pane 0 — rqt_image_view on the raw camera feed. Polls for Gazebo's
# camera topics indirectly by waiting on the base session's move_group.
PANE0=$(tmux list-panes -t "$SESSION:$WINDOW" -F "#{pane_id}")
tmux send-keys -t "$PANE0" \
"$SHELL_DIR/wait_for_node.sh move_group 30 && source ~/ros2_ws/install/setup.bash && ros2 run rqt_image_view rqt_image_view /wrist_rgbd_depth_sensor/image_raw" C-m

# Pane 1 — capture_camera.py, left at a ready prompt (not auto-started —
# capture cadence/target dir is a per-session choice, see
# capture_camera.py's --env/--out/--count/--every).
PANE1=$(tmux split-window -t "$PANE0" -h -P -F "#{pane_id}")
tmux send-keys -t "$PANE1" \
"cd $PYTHON_DIR && source ~/ros2_ws/install/setup.bash && echo 'Ready. Run: python3 capture_camera.py --env sim --out ~/captures --count 10'" C-m

# Pane 2 — isolated YOLO venv, ready prompt. Deliberately does NOT source
# ROS's setup.bash — this pane is for YOLO-only work (ultralytics import,
# inference on saved images from pane 1), kept out of any process that
# also imports cv_bridge.
PANE2=$(tmux split-window -t "$PANE0" -v -P -F "#{pane_id}")
tmux send-keys -t "$PANE2" \
"source $SHELL_DIR/aliases.sh && yoloenv && echo 'YOLO venv active. Run: python3 -c \"from ultralytics import YOLO; print(1)\"' || echo 'No venv yet — run installyolo first.'" C-m

# Pane 3 — free scratch pane, ROS-sourced, for ad-hoc topic echo/debug.
PANE3=$(tmux split-window -t "$PANE1" -v -P -F "#{pane_id}")
tmux send-keys -t "$PANE3" \
"source ~/ros2_ws/install/setup.bash && source $SHELL_DIR/aliases.sh" C-m

# Give each pane a title
tmux select-pane -t "$PANE0" -T "Camera View"
tmux select-pane -t "$PANE1" -T "Capture"
tmux select-pane -t "$PANE2" -T "YOLO venv"
tmux select-pane -t "$PANE3" -T "Scratch"

# Show pane titles on pane borders (must be set after the panes above
# exist — setting it earlier has no target to render into).
tmux set-option -t "$SESSION" pane-border-status top
tmux set-option -t "$SESSION" pane-border-format "#{?pane_active,#[fg=green]▶ ,}#{pane_title}"

tmux select-layout -t "$SESSION:$WINDOW" tiled

# Attach at the end
tmux select-window -t "$SESSION:$WINDOW"
tmux attach-session -t "$SESSION"
