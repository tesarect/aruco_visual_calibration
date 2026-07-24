#!/bin/bash
# Fail-fast sibling of check_real_driver.sh — checks controller_manager,
# /joint_states, and /tf in order, and exits immediately (with an echo)
# on the FIRST failure instead of running every check to completion and
# printing a full report. Used only as a startup gate (see
# real_tmux_base.sh's stat_check=on) where you just want a fast yes/no,
# not the full diagnostic — for the full report, run check_real_driver.sh
# directly (unchanged, still the mid-session tool).
#
# Exit code: 0 if controller_manager, /joint_states, and /tf are all
# healthy; 1 on the first one that isn't (message says which).

source ~/ros2_ws/install/setup.bash 2>/dev/null

echo "Quick driver check (fail-fast — controller_manager, /joint_states, /tf)..."

NODES=$(ros2 node list 2>/dev/null)
if ! echo "$NODES" | grep -qE "controller_manager"; then
    echo "MISSING  controller_manager (ros2_control) — stopping check here."
    exit 1
fi
echo "OK   controller_manager (ros2_control)"

if ! timeout 3 ros2 topic hz /joint_states --window 5 2>&1 | grep -q "average rate"; then
    echo "SILENT / NOT FOUND  /joint_states — stopping check here."
    exit 1
fi
echo "OK   /joint_states PUBLISHING"

if ! timeout 3 ros2 topic hz /tf --window 5 2>&1 | grep -q "average rate"; then
    echo "SILENT / NOT FOUND  /tf — stopping check here."
    exit 1
fi
echo "OK   /tf PUBLISHING"

echo "READY: controller_manager/joint_states/tf all healthy."
exit 0
