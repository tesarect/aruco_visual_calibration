#!/usr/bin/env python3
"""Moves the arm to a NAMED preset pose (e.g. "home", "standby",
"baristastandby", or any other entry under preset_names in
preset_poses_sim.yaml/_real.yaml), without needing to copy that preset's
raw xyz/quat by hand into a ~/trace_path call.

Two-step glue: looks up the preset via trajectory_planner's
~/get_preset_pose (read-only — see GetPresetPose.srv), then sends the
returned Pose to ~/trace_path (see TracePath.srv) to actually move there.
Both calls go through trajectory_planner_node — no direct pose math here,
so it stays consistent with whatever's actually recorded in
preset_poses_*.yaml (see pose_capture.py record for how those get added).

Usage:
    python3 move_to_preset.py baristastandby
    python3 move_to_preset.py home --joint-space
    python3 move_to_preset.py standby --cartesian

Defaults to joint-space planning (safer/more robust — see TracePath.srv's
planning_mode doc comment); pass --cartesian for a straight-line move
instead.

Nodes that must already be running:
    - trajectory_planner_node
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from visual_calibration_msgs.srv import GetPresetPose, TracePath


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("name", help="Preset name, e.g. baristastandby")
    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument(
        "--joint-space", action="store_true", default=True,
        help="Plan via free-space joint-space planning (default).")
    mode_group.add_argument(
        "--cartesian", action="store_true",
        help="Plan via straight-line Cartesian interpolation instead.")
    args = parser.parse_args()

    rclpy.init()
    node = Node("move_to_preset")

    get_preset_client = node.create_client(
        GetPresetPose, "/trajectory_planner/get_preset_pose")
    trace_path_client = node.create_client(
        TracePath, "/trajectory_planner/trace_path")

    if not get_preset_client.wait_for_service(timeout_sec=5.0) or \
            not trace_path_client.wait_for_service(timeout_sec=5.0):
        print("ERROR: trajectory_planner services not available — "
              "is trajectory_planner_node running?", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)

    preset_request = GetPresetPose.Request()
    preset_request.name = args.name
    preset_future = get_preset_client.call_async(preset_request)
    rclpy.spin_until_future_complete(node, preset_future, timeout_sec=10.0)
    preset_result = preset_future.result()

    if preset_result is None:
        print("ERROR: ~/get_preset_pose call timed out.", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)
    if not preset_result.success:
        print(f"ERROR: {preset_result.message}", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)

    print(f"Found preset '{args.name}': {preset_result.message}")

    trace_request = TracePath.Request()
    trace_request.waypoints = [preset_result.pose]
    trace_request.planning_mode = (
        TracePath.Request.PLANNING_MODE_CARTESIAN
        if args.cartesian else TracePath.Request.PLANNING_MODE_JOINT_SPACE
    )
    trace_request.pose_name = args.name
    trace_future = trace_path_client.call_async(trace_request)
    rclpy.spin_until_future_complete(node, trace_future, timeout_sec=30.0)
    trace_result = trace_future.result()

    if trace_result is None:
        print("ERROR: ~/trace_path call timed out.", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)
    if not trace_result.success:
        print(f"ERROR: {trace_result.message}", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)

    print(f"Moved to '{args.name}': {trace_result.message}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()