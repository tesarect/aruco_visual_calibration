#!/bin/bash
# Jenkins "Gazebo (sim)" stage — first of the four stages that used to be
# one monolithic stage_base_sim.sh (split up so a Gazebo-only port clash
# doesn't have to drag move_group/planning_scene through the same retry,
# and so Jenkins' UI shows exactly which layer failed). Launches Gazebo,
# waits for the robot to spawn and joint_state_broadcaster to go active —
# i.e. everything through controller activation, since spawners depend on
# Gazebo coming up within the same few seconds and aren't worth a separate
# stage/retry unit. See stage_movegroup_sim.sh / stage_planning_scene_sim.sh
# for the rest of the former stage_base_sim.sh chain.
#
# Retried (2 retries, 3 attempts total) by the Jenkinsfile specifically at
# THIS stage — confirmed live failure mode: a stale gzserver from a prior
# run/attempt still holding port 11345 causes gzserver to die immediately
# ("Address already in use"), which cascades into every spawner and
# robot_state_publisher failing since none of them have anything to attach
# to. Not a code bug — a clean restart with the port actually freed fixes
# it, hence retry instead of a one-shot failure.

set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./pipeline_common.sh
source "$SCRIPT_DIR/pipeline_common.sh"

RESOURCES_SHELL_DIR="$HOME/ros2_ws/src/visual_calibration/resources/scripts/shell"

# colcon's generated setup.bash references its own internal vars (e.g.
# COLCON_TRACE) without defaulting them — fatal under this script's own
# `set -u`. See stage_base_sim.sh's original header note for the full
# explanation; same fix here.
set +u
source ~/ros2_ws/install/setup.bash
set -u

# Cleanup BEFORE launching — covers both a previous FAILED attempt of this
# same stage (retry re-runs this whole script) and any stray process left
# by an entirely separate prior run/session. kill_tracked_pids first (this
# build's own tracked processes), then kill_stray_ros_processes (anything
# untracked, e.g. from a manual `ros2 launch` outside Jenkins) — the latter
# now blocks until port 11345 is actually free (see pipeline_common.sh).
echo "=== [stage_gazebo_sim] Cleaning up any processes from a previous attempt/prior run first ==="
kill_tracked_pids
kill_stray_ros_processes

echo "=== [stage_gazebo_sim] Starting Gazebo (starbots_ur3e.launch.xml) ==="
ros2 launch the_construct_office_gazebo starbots_ur3e.launch.xml \
    > "$LOG_DIR/base_sim_gazebo.log" 2>&1 &
track_pid "$!" gazebo

echo "=== [stage_gazebo_sim] Waiting for joint_state_broadcaster active ==="
"$RESOURCES_SHELL_DIR/wait_for_controllers.sh" /controller_manager 90
CONTROLLER_STATUS=$?
if [ "$CONTROLLER_STATUS" -ne 0 ]; then
    echo "[stage_gazebo_sim] controller_manager never reported joint_state_broadcaster active — failing stage."
    exit 1
fi

echo "=== [stage_gazebo_sim] Gazebo (sim) stage complete ==="
