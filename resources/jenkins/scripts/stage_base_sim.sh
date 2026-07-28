#!/bin/bash
# Jenkins "Base (sim)" stage — replicates sim_tmux_base.sh's Gazebo +
# move_group + planning_scene_setup chain as backgrounded processes with
# readiness gates in THIS shell step (not a separate stage assuming
# readiness — see error-mitigation.md #17/#36 and the design doc's warning
# about racing a `sh` step that backgrounds something and returns
# immediately). RViz is intentionally NOT started by default — headless
# CI/demo run, no display needed — but CAN be started, along with the
# rqt debug tools from debug_tmux.sh, via DEVDEBUG=true (see below); the
# rosject has a VNC/remote-desktop view these are actually visible in, so
# this isn't launching blind.
#
# Does NOT use sim_tmux_base.sh itself (that's the tmux/dev path, untouched
# by this work) — replicates its commands directly per the design doc,
# since bringup_full_sim's chain explicitly excludes Gazebo/move_group.

set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./pipeline_common.sh
source "$SCRIPT_DIR/pipeline_common.sh"

RESOURCES_SHELL_DIR="$HOME/ros2_ws/src/visual_calibration/resources/scripts/shell"
RESOURCES_PYTHON_DIR="$HOME/ros2_ws/src/visual_calibration/resources/scripts/python"

# colcon's generated setup.bash references its own internal vars (e.g.
# COLCON_TRACE) without defaulting them — fine under a normal shell, but
# fatal under this script's own `set -u` (inherited from
# pipeline_common.sh), which turns any unset-variable reference into a
# hard error. Confirmed live: "setup.bash: line 11: COLCON_TRACE: unbound
# variable" killed this stage before it ran a single ros2 command.
# Disable -u for just this one sourced file (not ours to fix/control),
# then restore it immediately after for the rest of this script.
set +u
source ~/ros2_ws/install/setup.bash
set -u

# DEVDEBUG (env var, set by the Jenkinsfile from its DEVDEBUG boolean
# parameter — default false): when true, also starts RViz (from
# sim_tmux_base.sh's own pane 2) plus the three debug_tmux.sh tools
# (tf_debug_markers.py, rqt_image_view on the ArUco overlay topic,
# rqt_graph) as backgrounded, PID-tracked processes — same commands those
# two tmux scripts use, just without tmux itself (Jenkins has no terminal
# multiplexer to attach to; the rosject's VNC/remote-desktop view is where
# these actually show up). Defaults to false so a normal/headless pipeline
# run doesn't pay the extra startup cost or clutter the desktop.
DEVDEBUG="${DEVDEBUG:-false}"

# Proactive stale-process sweep BEFORE launching Gazebo — confirmed live
# root cause of a real failure: gzserver failed to bind its master port
# ("EXCEPTION: Unable to start server[bind: Address already in use].
# There is probably another Gazebo process running.") because a PRIOR
# run's gzserver was still alive and holding it. pipeline_common.sh's
# kill_stray_ros_processes existed but was only ever called in the
# pipeline's final `post` cleanup — too late to help the NEXT run start
# cleanly. Root cause of the leftover process itself: gzserver/gzclient
# are children of the `ros2 launch` process this script backgrounds, not
# separate PIDs track_pid captures — an abrupt stop (e.g. a manually
# aborted Jenkins build) can leave them orphaned and still bound to the
# port even though the tracked launch-wrapper PID is gone. Sweeping here
# too, not just in `post`, closes that gap for the very next run.
#
# kill_tracked_pids too (not just kill_stray_ros_processes): needed now
# that the Jenkinsfile wraps this stage in retry(3) for transient
# failures (e.g. a spawner — gripper_controller/joint_state_broadcaster/
# joint_trajectory_controller — occasionally dying with exit code 1 for
# no logged reason, confirmed happening intermittently). Without this, a
# FAILED first attempt's own move_group/Gazebo (tracked in THIS SAME
# workspace's .pids file, since LOG_DIR is per-build-number but retry()
# re-runs within the same build) would still be alive and interfere with
# the next attempt — this isn't cross-build cleanup (that's `post`'s job),
# it's cross-ATTEMPT cleanup within one retrying build.
echo "=== [stage_base_sim] Cleaning up any processes from a previous attempt/prior run first ==="
kill_tracked_pids
kill_stray_ros_processes

echo "=== [stage_base_sim] Starting Gazebo (starbots_ur3e.launch.xml) ==="
source ~/ros2_ws/install/setup.bash
ros2 launch the_construct_office_gazebo starbots_ur3e.launch.xml \
    > "$LOG_DIR/base_sim_gazebo.log" 2>&1 &
track_pid "$!" gazebo

echo "=== [stage_base_sim] Waiting for joint_state_broadcaster active ==="
"$RESOURCES_SHELL_DIR/wait_for_controllers.sh" /controller_manager 90
CONTROLLER_STATUS=$?
if [ "$CONTROLLER_STATUS" -ne 0 ]; then
    echo "[stage_base_sim] controller_manager never reported joint_state_broadcaster active — failing stage."
    exit 1
fi

echo "=== [stage_base_sim] Starting move_group (sim_ur3e_moveit_config) ==="
source ~/ros2_ws/install/setup.bash
ros2 launch sim_ur3e_moveit_config move_group.launch.py \
    > "$LOG_DIR/base_sim_move_group.log" 2>&1 &
track_pid "$!" move_group

# 60s consistently wasn't enough under Jenkins (confirmed live, build #29:
# Gazebo/controllers came up cleanly and fast — 1s — but move_group still
# timed out at 60s with no error in its own log, just still loading).
# sim_tmux_base.sh's tmux equivalent only waits 30s, but that's a human
# watching an interactive pane who'd just wait longer by eye if it was
# still loading — Jenkins has no such fallback, a timeout here is a hard
# stage failure. Raised to 180s: move_group loading kinematics/planning
# pipeline plugins can legitimately take longer when Jenkins, colcon,
# Gazebo, and the Grafana/Loki/Promtail stack are all competing for the
# same rosject's resources at once, which an interactive dev session
# typically isn't doing at the same time.
echo "=== [stage_base_sim] Waiting for move_group node ==="
"$RESOURCES_SHELL_DIR/wait_for_node.sh" move_group 30
if [ $? -ne 0 ]; then
    echo "[stage_base_sim] move_group never came up — failing stage."
    exit 1
fi

echo "=== [stage_base_sim] Running planning_scene_setup (one-shot populate) ==="
ros2 launch visual_calibration_moveit planning_scene_setup.launch.py env:=sim \
    > "$LOG_DIR/base_sim_planning_scene.log" 2>&1 &
track_pid "$!" planning_scene_setup

echo "=== [stage_base_sim] Waiting for planning scene to contain countertop+wall ==="
"$RESOURCES_SHELL_DIR/wait_for_planning_scene.sh" 30 2
if [ $? -ne 0 ]; then
    echo "[stage_base_sim] planning scene not confirmed populated within timeout — continuing (same 'continuing anyway' convention as the underlying script), but this is a strong signal to check base_sim_planning_scene.log."
fi

if [ "$DEVDEBUG" = "true" ]; then
    echo "=== [stage_base_sim] DEVDEBUG=true — starting RViz + debug tools (visible via rosject VNC) ==="

    echo "--- RViz (sim_ur3e_moveit_config) ---"
    ros2 launch sim_ur3e_moveit_config moveit_rviz.launch.py \
        > "$LOG_DIR/base_sim_rviz.log" 2>&1 &
    track_pid "$!" rviz

    echo "--- tf_debug_markers.py (polls for move_group itself) ---"
    python3 "$RESOURCES_PYTHON_DIR/tf_debug_markers.py" --env sim \
        > "$LOG_DIR/base_sim_tf_debug_markers.log" 2>&1 &
    track_pid "$!" tf_debug_markers

    echo "--- rqt_image_view (ArUco overlay topic) ---"
    ros2 run rqt_image_view rqt_image_view /aruco_perception/overlay_image \
        > "$LOG_DIR/base_sim_rqt_image_view.log" 2>&1 &
    track_pid "$!" rqt_image_view

    echo "--- rqt_graph ---"
    ros2 run rqt_graph rqt_graph \
        > "$LOG_DIR/base_sim_rqt_graph.log" 2>&1 &
    track_pid "$!" rqt_graph
fi

echo "=== [stage_base_sim] Base (sim) stage complete ==="
