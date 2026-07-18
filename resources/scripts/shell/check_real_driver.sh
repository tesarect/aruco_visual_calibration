#!/bin/bash
# Dirty, fast "is the real robot stack actually alive" check — run this
# immediately after entering the rosject, before launching MoveIt/RViz, to
# avoid repeating the "move_group up but robot_description/tf never
# arrives because the driver was never (re)started" failure mode.
#
# Not a launcher — the UR driver/ros2_control bringup lives on the robot's
# own side (per CLAUDE.md's Real Robot Camera Setup), this only reports
# status so you know whether to go start it first.

source ~/ros2_ws/install/setup.bash 2>/dev/null

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
check_node "controller_manager" "controller_manager (ros2_control)"
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
timeout 3 ros2 topic hz /joint_states --window 5 2>&1 | grep -q "average rate" \
    && echo "PUBLISHING" || echo "SILENT / NOT FOUND"

echo -n "/tf: "
timeout 3 ros2 topic hz /tf --window 5 2>&1 | grep -q "average rate" \
    && echo "PUBLISHING" || echo "SILENT / NOT FOUND"

echo
echo "=== TF world frame ==="
timeout 3 ros2 run tf2_ros tf2_echo world base_link 2>&1 | head -3

echo
echo "=== Controllers ==="
timeout 5 ros2 control list_controllers 2>&1

echo
echo "=== Camera (D415 / Zenoh bridge) ==="
timeout 3 ros2 topic hz /D415/color/image_raw --window 5 2>&1 | grep -q "average rate" \
    && echo "D415 color: PUBLISHING" || echo "D415 color: SILENT / NOT FOUND"

echo
echo "--- If joint_states/tf are SILENT: the real robot driver was not"
echo "    (re)started this session — go start it via the lab's robot"
echo "    bringup procedure before launching move_group/RViz."