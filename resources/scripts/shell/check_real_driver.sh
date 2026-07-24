#!/bin/bash
# Dirty, fast "is the real robot stack actually alive" check — run this
# immediately after entering the rosject, before launching MoveIt/RViz, to
# avoid repeating the "move_group up but robot_description/tf never
# arrives because the driver was never (re)started" failure mode.
#
# Not a launcher — the UR driver/ros2_control bringup lives on the robot's
# own side (per CLAUDE.md's Real Robot Camera Setup), this only reports
# status so you know whether to go start it first.
#
# Exit code doubles as a machine-checkable pass/fail (see real_tmux_base.sh,
# which refuses to launch anything if this fails): 0 if controller_manager,
# joint_states, and tf are all healthy; 1 otherwise. Camera/D415 and the
# individual node-presence checks are report-only and do NOT affect the
# exit code — they're informational, not blockers for MoveIt/RViz.

source ~/ros2_ws/install/setup.bash 2>/dev/null

READY=1

echo "=== Nodes ==="
NODES=$(ros2 node list 2>/dev/null)
echo "$NODES"
echo

check_node() {
    local pattern="$1"
    local label="$2"
    if echo "$NODES" | grep -qE "$pattern"; then
        echo "OK   $label"
    else
        echo "MISSING  $label"
    fi
}

echo "=== Driver stack ==="
check_node "robot_state_publisher" "robot_state_publisher"
if echo "$NODES" | grep -qE "controller_manager"; then
    echo "OK   controller_manager (ros2_control)"
else
    echo "MISSING  controller_manager (ros2_control)"
    READY=0
fi
# Pattern updated 2026-07-18: the original ur_ros2_control_node|ur_robot_driver
# pattern never matched this driver version's actual node names — a live
# session showed /ur_tool_comm, /ur_configuration_controller,
# /urscript_interface, /dashboard_client, /ur_robot_state_helper instead,
# causing a false "MISSING" even though the driver (confirmed via active
# scaled_joint_trajectory_controller + publishing /joint_states and /tf)
# was actually fine. Match on any of the driver-specific node names
# instead of guessing a single expected name.
check_node "ur_tool_comm|ur_configuration_controller|urscript_interface|ur_robot_state_helper" "UR driver"

echo
echo "=== Topics ==="
echo -n "/joint_states: "
if timeout 3 ros2 topic hz /joint_states --window 5 2>&1 | grep -q "average rate"; then
    echo "PUBLISHING"
else
    echo "SILENT / NOT FOUND"
    READY=0
fi

echo -n "/tf: "
if timeout 3 ros2 topic hz /tf --window 5 2>&1 | grep -q "average rate"; then
    echo "PUBLISHING"
else
    echo "SILENT / NOT FOUND"
    READY=0
fi

echo
echo "=== TF world frame ==="
timeout 3 ros2 run tf2_ros tf2_echo world base_link 2>&1 | head -3

echo
echo "=== Controllers ==="
CONTROLLERS_OUT=$(timeout 5 ros2 control list_controllers 2>&1)
echo "$CONTROLLERS_OUT"
if [ -z "$CONTROLLERS_OUT" ]; then
    READY=0
fi

echo
echo "=== Camera (D415 / Zenoh bridge) ==="
timeout 3 ros2 topic hz /D415/color/image_raw --window 5 2>&1 | grep -q "average rate" \
    && echo "D415 color: PUBLISHING" || echo "D415 color: SILENT / NOT FOUND"

echo
if [ "$READY" -eq 1 ]; then
    echo "=== READY: controller_manager/joint_states/tf all healthy ==="
else
    echo "--- NOT READY: controller_manager, joint_states, or tf is missing/"
    echo "    silent. The real robot driver was not (re)started this"
    echo "    session — go start it via the lab's robot bringup procedure"
    echo "    before launching move_group/RViz."
fi

exit $((1 - READY))