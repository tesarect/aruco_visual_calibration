#!/bin/bash
# Polls inference_server.py's GET /health until it responds, or a timeout
# elapses. This is the non-ROS equivalent of
# resources/scripts/shell/wait_for_node.sh (which polls `ros2 node list` —
# useless here, since inference_server.py is a plain Flask process, never a
# ROS node) — same structure/output convention, just a different readiness
# signal.
#
# health also reports which env's model is loaded (see inference_server.py's
# /health handler), which this script surfaces so a mismatched
# start_inference_server.sh env arg is obvious rather than a silent wrong
# result later.
#
# Usage: wait_for_inference_server.sh [timeout_sec] [expected_env]
#   (expected_env optional — if given, mismatches are warned about but do
#   NOT fail the wait, since a running server of the wrong env is still
#   "ready", just not what the caller probably wanted)

TIMEOUT_SEC="${1:-30}"
EXPECTED_ENV="${2:-}"
HEALTH_URL="http://127.0.0.1:8600/health"
ELAPSED=0

echo "Waiting for inference_server.py at $HEALTH_URL (timeout ${TIMEOUT_SEC}s)..."
while true; do
    RESPONSE=$(curl -s -m 2 "$HEALTH_URL" 2>/dev/null)
    if [ -n "$RESPONSE" ] && echo "$RESPONSE" | grep -q '"status": *"ok"'; then
        echo "inference_server.py is up (waited ${ELAPSED}s). Response: $RESPONSE"
        if [ -n "$EXPECTED_ENV" ] && ! echo "$RESPONSE" | grep -q "\"env\": *\"$EXPECTED_ENV\""; then
            echo " ⚠️  Expected env '$EXPECTED_ENV' but the running server reports a different "
            echo "    env — check which start_inference_server.sh call actually launched it."
        fi
        exit 0
    fi
    sleep 1
    ELAPSED=$((ELAPSED + 1))
    if [ "$ELAPSED" -ge "$TIMEOUT_SEC" ]; then
        echo "Timed out waiting for inference_server.py after ${TIMEOUT_SEC}s — continuing anyway."
        exit 1
    fi
done