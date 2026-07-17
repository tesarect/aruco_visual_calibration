#!/bin/bash
# Polls `ros2 control list_controllers` until joint_state_broadcaster
# reports active, or a timeout elapses. This is a real readiness signal
# (Gazebo's gazebo_ros2_control plugin has finished loading hardware AND
# controller_manager has activated the broadcaster) — unlike
# wait_for_node.sh (only checks the node process exists, not that
# controllers finished loading) or a fixed `sleep N` (races against
# variable Gazebo startup time, sometimes not long enough). Used to gate
# move_group/rviz starting in sim_tmux_base.sh so they don't connect to a
# controller_manager that's still mid-load.
#
# Usage: wait_for_controllers.sh [controller_manager] [timeout_sec]
source ~/ros2_ws/install/setup.bash

CM="${1:-/controller_manager}"
TIMEOUT_SEC="${2:-60}"
ELAPSED=0

echo "Waiting for joint_state_broadcaster to be active on '$CM' (timeout ${TIMEOUT_SEC}s)..."
while true; do
    STATE=$(ros2 control list_controllers -c "$CM" 2>/dev/null | \
        sed -E 's/\[[^]]*\]//' | \
        awk '$1 == "joint_state_broadcaster" {print $2}')

    if [ "$STATE" = "active" ]; then
        echo "joint_state_broadcaster is active (waited ${ELAPSED}s)."
        exit 0
    fi

    sleep 1
    ELAPSED=$((ELAPSED + 1))
    if [ "$ELAPSED" -ge "$TIMEOUT_SEC" ]; then
        echo "Timed out waiting for joint_state_broadcaster after ${TIMEOUT_SEC}s — continuing anyway."
        exit 1
    fi
done
