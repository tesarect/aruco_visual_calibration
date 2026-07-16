#!/bin/bash

CM="$1"
CONTROLLER_NAME="$2"

if [ $# -ne 2 ]; then
    echo "Usage: $0 <controller_manager> <controller_name>"
    exit 1
fi

STATE=$(ros2 control list_controllers -c "$CM" | \
    awk -v ctrl="$CONTROLLER_NAME" '$1 == ctrl {print $3}')

if [ -z "$STATE" ]; then
    echo "Controller '$CONTROLLER_NAME' not found on '$CM'"
    exit 1
fi

if [ "$STATE" = "active" ]; then
    echo "Controller '$CONTROLLER_NAME' is already active."
    exit 0
fi

echo "Activating '$CONTROLLER_NAME'..."
ros2 control switch_controllers \
    -c "$CM" \
    --activate "$CONTROLLER_NAME"
