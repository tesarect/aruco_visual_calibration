#!/bin/bash
# Polls `ros2 node list` until a node matching the given name appears, or a
# timeout elapses. Node presence is a simple readiness proxy (process
# started successfully) — not a guarantee the node has finished its own
# internal setup (e.g. move_group may still be loading the planning scene).
#
# Usage: wait_for_node.sh <node_name_substring> [timeout_sec]
source ~/ros2_ws/install/setup.bash

NODE_NAME="${1:?wait_for_node.sh requires a node name}"
TIMEOUT_SEC="${2:-30}"
ELAPSED=0

echo "Waiting for node matching '$NODE_NAME' (timeout ${TIMEOUT_SEC}s)..."
while ! ros2 node list 2>/dev/null | grep -q "$NODE_NAME"; do
    sleep 1
    ELAPSED=$((ELAPSED + 1))
    if [ "$ELAPSED" -ge "$TIMEOUT_SEC" ]; then
        echo "Timed out waiting for node '$NODE_NAME' after ${TIMEOUT_SEC}s — continuing anyway."
        exit 1
    fi
done
echo "Node matching '$NODE_NAME' is up (waited ${ELAPSED}s)."