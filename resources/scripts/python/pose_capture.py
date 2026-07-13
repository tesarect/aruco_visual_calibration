#!/usr/bin/env python3
"""Real-robot quick tool: record the current end-effector pose to a text
file, or compute a candidate standoff pose to jog the arm to by hand before
running detection scripts.

There is no camera_frame TF on the real robot yet (that's what this whole
project is computing) — so `standoff` here does NOT look up a camera TF
like trajectory_planner_node does. Instead it takes a reference frame you
already trust (default: base_link) and an explicit offset you supply, and
prints/saves the resulting rg2_gripper_aruco_link target pose in base_link.
Use this to get the arm roughly in front of wherever you've physically
placed the camera, then fine-tune by hand/teach-pendant.

Usage:
    # Record the current rg2_gripper_aruco_link pose (base_link-relative)
    # to a text file — run this once you've manually jogged the arm to a
    # pose where the camera can see the marker.
    python3 pose_capture.py record --out ~/poses.txt

    # Compute a standoff pose: 0.3m along +Z, 0.0/0.0m along X/Y, from
    # base_link's origin, facing back along -Z (roll=pi so the marker
    # faces the camera placed further along +Z). Prints the target pose
    # and, with --move, sends it to trajectory_planner's ~/trace_path.
    python3 pose_capture.py standoff --z 0.3 --roll 3.14159265 --out ~/standoff_pose.txt
    python3 pose_capture.py standoff --z 0.3 --roll 3.14159265 --move

Nodes that must already be running:
    - Real robot driver / joint_state_broadcaster publishing TF (for `record`)
    - trajectory_planner_node (only if `standoff --move` is used)
"""

import argparse
import datetime
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Pose
from visual_calibration_msgs.srv import TracePath

EE_FRAME = "rg2_gripper_aruco_link"
BASE_FRAME = "base_link"


def lookup_pose(node, tf_buffer, target_frame, source_frame, timeout_sec=5.0):
    deadline = node.get_clock().now() + rclpy.duration.Duration(seconds=timeout_sec)
    while node.get_clock().now() < deadline:
        try:
            return tf_buffer.lookup_transform(source_frame, target_frame, Time())
        except TransformException:
            rclpy.spin_once(node, timeout_sec=0.2)
    raise RuntimeError(
        f"Could not look up {source_frame} -> {target_frame} within {timeout_sec}s "
        "(is the robot driver / TF publisher running?)"
    )


def format_pose_line(label, tf):
    t = tf.transform.translation
    q = tf.transform.rotation
    roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    stamp = datetime.datetime.now().isoformat(timespec="seconds")
    return (
        f"[{stamp}] {label}: "
        f"xyz=({t.x:.4f}, {t.y:.4f}, {t.z:.4f}) "
        f"quat=({q.x:.4f}, {q.y:.4f}, {q.z:.4f}, {q.w:.4f}) "
        f"rpy_rad=({roll:.4f}, {pitch:.4f}, {yaw:.4f})\n"
    )


def cmd_record(args):
    rclpy.init()
    node = Node("pose_capture_record")
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)

    try:
        tf = lookup_pose(node, tf_buffer, EE_FRAME, BASE_FRAME, args.timeout)
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)

    line = format_pose_line(f"{BASE_FRAME} -> {EE_FRAME}", tf)
    print(line.strip())
    with open(args.out, "a") as f:
        f.write(line)
    print(f"Appended to {args.out}")

    rclpy.shutdown()


def cmd_standoff(args):
    q = quaternion_from_euler(args.roll, args.pitch, args.yaw)
    pose = Pose()
    pose.position.x = args.x
    pose.position.y = args.y
    pose.position.z = args.z
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]

    stamp = datetime.datetime.now().isoformat(timespec="seconds")
    line = (
        f"[{stamp}] standoff target ({BASE_FRAME}-relative, for {EE_FRAME}): "
        f"xyz=({pose.position.x:.4f}, {pose.position.y:.4f}, {pose.position.z:.4f}) "
        f"quat=({q[0]:.4f}, {q[1]:.4f}, {q[2]:.4f}, {q[3]:.4f}) "
        f"rpy_rad=({args.roll:.4f}, {args.pitch:.4f}, {args.yaw:.4f})\n"
    )
    print(line.strip())
    if args.out:
        with open(args.out, "a") as f:
            f.write(line)
        print(f"Appended to {args.out}")

    if not args.move:
        print("Not moving (pass --move to send this pose to ~/trace_path). "
              "Jog the arm there manually first, confirm the marker is in "
              "view, then re-run with --move if you want the script to "
              "finish the approach itself.")
        return

    rclpy.init()
    node = Node("pose_capture_standoff")
    client = node.create_client(TracePath, "/trajectory_planner/trace_path")
    if not client.wait_for_service(timeout_sec=5.0):
        print("ERROR: /trajectory_planner/trace_path not available — "
              "is trajectory_planner_node running?", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)

    request = TracePath.Request()
    request.waypoints = [pose]
    request.planning_mode = (
        TracePath.Request.PLANNING_MODE_JOINT_SPACE
        if args.joint_space else TracePath.Request.PLANNING_MODE_CARTESIAN
    )
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
    result = future.result()
    if result is None:
        print("ERROR: ~/trace_path call timed out.", file=sys.stderr)
    elif not result.success:
        print(f"~/trace_path failed: {result.message}", file=sys.stderr)
    else:
        print(f"Moved to standoff pose: {result.message}")

    rclpy.shutdown()


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    p_record = sub.add_parser("record", help="Record current EE pose to a text file")
    p_record.add_argument("--out", default="poses.txt", help="Output text file (appended)")
    p_record.add_argument("--timeout", type=float, default=5.0, help="TF lookup timeout (s)")
    p_record.set_defaults(func=cmd_record)

    p_standoff = sub.add_parser(
        "standoff", help="Compute (and optionally move to) a candidate standoff pose")
    p_standoff.add_argument("--x", type=float, default=0.0, help=f"X offset from {BASE_FRAME} (m)")
    p_standoff.add_argument("--y", type=float, default=0.0, help=f"Y offset from {BASE_FRAME} (m)")
    p_standoff.add_argument("--z", type=float, default=0.3, help=f"Z offset from {BASE_FRAME} (m)")
    p_standoff.add_argument("--roll", type=float, default=math.pi, help="Roll (rad)")
    p_standoff.add_argument("--pitch", type=float, default=0.0, help="Pitch (rad)")
    p_standoff.add_argument("--yaw", type=float, default=0.0, help="Yaw (rad)")
    p_standoff.add_argument("--out", default=None, help="Optional text file to append the pose to")
    p_standoff.add_argument("--move", action="store_true",
                             help="Send the pose to ~/trace_path instead of just printing it")
    p_standoff.add_argument("--joint-space", action="store_true",
                             help="Use joint-space planning instead of Cartesian for --move")
    p_standoff.set_defaults(func=cmd_standoff)

    return parser


if __name__ == "__main__":
    args = build_parser().parse_args()
    args.func(args)
