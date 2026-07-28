#!/bin/bash
# Jenkins "move_group (sim)" stage — second of the four stages split out of
# the former stage_base_sim.sh. Assumes Gazebo + controllers are already up
# (stage_gazebo_sim.sh ran immediately before this in the same Jenkins
# build/workspace, so $LOG_DIR/.pids already has gazebo tracked). NOT
# retried by the Jenkinsfile — its one observed live failure mode (build
# #29) was a slow-but-successful load under Jenkins' resource contention,
# not a crash a restart would help; see the 180s timeout note below.

set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./pipeline_common.sh
source "$SCRIPT_DIR/pipeline_common.sh"

RESOURCES_SHELL_DIR="$HOME/ros2_ws/src/visual_calibration/resources/scripts/shell"

set +u
source ~/ros2_ws/install/setup.bash
set -u

echo "=== [stage_movegroup_sim] Starting move_group (sim_ur3e_moveit_config) ==="
ros2 launch sim_ur3e_moveit_config move_group.launch.py \
    > "$LOG_DIR/base_sim_move_group.log" 2>&1 &
track_pid "$!" move_group

# 60s consistently wasn't enough under Jenkins (confirmed live, build #29:
# Gazebo/controllers came up cleanly and fast — 1s — but move_group still
# timed out at 60s with no error in its own log, just still loading, and
# had in fact fully come up moments later per its own log's "You can start
# planning now!" line). sim_tmux_base.sh's tmux equivalent only waits 30s,
# but that's a human watching an interactive pane who'd just wait longer by
# eye if it was still loading — Jenkins has no such fallback, a timeout
# here is a hard stage failure. Raised to 180s: move_group loading
# kinematics/planning pipeline plugins can legitimately take longer when
# Jenkins, colcon, Gazebo, and the Grafana/Loki/Promtail stack are all
# competing for the same rosject's resources at once, which an interactive
# dev session typically isn't doing at the same time.
echo "=== [stage_movegroup_sim] Waiting for move_group node ==="
"$RESOURCES_SHELL_DIR/wait_for_node.sh" move_group 180
if [ $? -ne 0 ]; then
    echo "[stage_movegroup_sim] move_group never came up — failing stage."
    exit 1
fi

echo "=== [stage_movegroup_sim] move_group (sim) stage complete ==="
