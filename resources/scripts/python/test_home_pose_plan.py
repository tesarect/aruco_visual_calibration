#!/usr/bin/env python3
"""Throwaway diagnostic: sends a pose to move_group's /move_action action
server directly, to check whether IK/OMPL can find ANY plan for it —
bypassing trajectory_planner/RViz entirely, so we know if the pose itself
is the problem or something in how trajectory_planner calls
MoveGroupInterface.

Calls /move_action directly (the same action MoveGroupInterface::plan()
uses internally in C++) rather than depending on moveit_commander/moveit_py
Python bindings, which aren't reliably available under ROS 2 Humble.

Usage:
    python3 test_home_pose_plan.py [--pose home|standby]

Prints the MoveGroup action result's error_code and, if it fails, the full
result message — same information move_group already logs, but isolated
from trajectory_planner's own code path.
"""

import argparse
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    MotionPlanRequest,
    PlanningOptions,
    PositionConstraint,
    OrientationConstraint,
)
from shape_msgs.msg import SolidPrimitive

# Exact values from preset_poses_sim.yaml.
POSES = {
    "home": ((-0.0245, 0.2882, 0.6940), (-0.4998, -0.4994, 0.5002, 0.5006)),
    "standby": ((-0.0628, 0.1691, 0.4499), (-0.5478, -0.4234, 0.2069, 0.6913)),
}
PLANNING_GROUP = "ur_manipulator"
END_EFFECTOR_FRAME = "rg2_gripper_aruco_link"
PLANNING_FRAME = "base_link"


def build_goal(position, orientation_xyzw):
    pose = PoseStamped()
    pose.header.frame_id = PLANNING_FRAME
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position
    (pose.pose.orientation.x, pose.pose.orientation.y,
     pose.pose.orientation.z, pose.pose.orientation.w) = orientation_xyzw

    position_constraint = PositionConstraint()
    position_constraint.header.frame_id = PLANNING_FRAME
    position_constraint.link_name = END_EFFECTOR_FRAME
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.SPHERE
    primitive.dimensions = [0.001]  # 1mm tolerance sphere, same order as MoveIt's own default
    position_constraint.constraint_region.primitives.append(primitive)
    position_constraint.constraint_region.primitive_poses.append(pose.pose)
    position_constraint.weight = 1.0

    orientation_constraint = OrientationConstraint()
    orientation_constraint.header.frame_id = PLANNING_FRAME
    orientation_constraint.link_name = END_EFFECTOR_FRAME
    orientation_constraint.orientation = pose.pose.orientation
    orientation_constraint.absolute_x_axis_tolerance = 0.001
    orientation_constraint.absolute_y_axis_tolerance = 0.001
    orientation_constraint.absolute_z_axis_tolerance = 0.001
    orientation_constraint.weight = 1.0

    constraints = Constraints()
    constraints.position_constraints.append(position_constraint)
    constraints.orientation_constraints.append(orientation_constraint)

    request = MotionPlanRequest()
    request.group_name = PLANNING_GROUP
    request.goal_constraints.append(constraints)
    request.num_planning_attempts = 10
    request.allowed_planning_time = 5.0

    goal = MoveGroup.Goal()
    goal.request = request
    goal.planning_options = PlanningOptions()
    goal.planning_options.plan_only = True  # plan only — do NOT execute
    return goal


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pose", choices=POSES.keys(), default="home")
    args = parser.parse_args()
    position, orientation_xyzw = POSES[args.pose]

    rclpy.init()
    node = Node("test_home_pose_plan")
    client = ActionClient(node, MoveGroup, "/move_action")

    print("Waiting for /move_action...")
    if not client.wait_for_server(timeout_sec=10.0):
        print("ERROR: /move_action not available.", file=sys.stderr)
        sys.exit(1)

    print(
        f"Sending plan-only goal: group='{PLANNING_GROUP}', "
        f"link='{END_EFFECTOR_FRAME}', pose='{args.pose}', position={position}, "
        f"orientation_xyzw={orientation_xyzw}")

    goal_future = client.send_goal_async(build_goal(position, orientation_xyzw))
    rclpy.spin_until_future_complete(node, goal_future)
    goal_handle = goal_future.result()

    if not goal_handle.accepted:
        print("Goal was REJECTED by move_group.")
        sys.exit(1)

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    result = result_future.result().result

    print(f"error_code.val = {result.error_code.val}")
    # moveit_msgs/MoveItErrorCodes: SUCCESS = 1, everything else is a
    # failure code (negative for most planning failures).
    if result.error_code.val == 1:
        print("SUCCESS — a valid plan was found for the exact 'home' pose.")
    else:
        print("FAILED — see error_code.val above (1 = success, anything else = failure; "
              "e.g. -1 = FAILURE, -31 = NO_IK_SOLUTION — see moveit_msgs/MoveItErrorCodes.msg "
              "on this system for the full list).")

    rclpy.shutdown()


if __name__ == "__main__":
    main()