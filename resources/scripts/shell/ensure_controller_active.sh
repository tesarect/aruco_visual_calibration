#!/bin/bash

CM="$1"
CONTROLLER_NAME="$2"

if [ $# -ne 2 ]; then
    echo "Usage: $0 <controller_manager> <controller_name>"
    exit 1
fi

# ros2 control list_controllers output has no space between the
# controller name and its [type] (e.g.
# "scaled_joint_trajectory_controller[ur_controllers/ScaledJointTrajectoryController] active"),
# so awk's default whitespace field-splitting puts name+type together in
# $1 — matching bare CONTROLLER_NAME against $1 never succeeds. Strip
# everything from '[' onward on each line before comparing, and read
# state from $2 once the line is reduced to "name state".
STATE=$(ros2 control list_controllers -c "$CM" | \
    sed -E 's/\[[^]]*\]//' | \
    awk -v ctrl="$CONTROLLER_NAME" '$1 == ctrl {print $2}')

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
