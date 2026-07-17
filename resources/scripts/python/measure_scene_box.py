#!/usr/bin/env python3
"""Real-robot quick tool: measure a box-shaped planning-scene object
(coffee machine, cupholder, countertop, wall, ...) by jogging the arm's
end-effector (tool0) to two opposite corners of its bounding box via
RViz's interactive marker, recording each corner's base_link-relative
position, then computing the box's center pose + size — ready to paste
into scene_objects_real.yaml.

Why tool0, not a marker link: this script only needs a physical point to
touch, not a detected/measured link — tool0 exists on every environment
(sim and real) with no dependency on the (not-yet-measured, see todo.txt
Thread B2) marker link. Jog with RViz's MotionPlanning interactive marker,
plan+execute (or drag freely and just record — this script only reads the
CURRENT TF, no motion is triggered), then run `corner` for each of the two
opposite corners.

Usage:
    # After jogging the arm's tool0 to touch the object's near-bottom
    # corner (any two opposite corners work, order doesn't matter):
    python3 measure_scene_box.py corner --out ~/coffee_machine_corners.txt

    # Repeat for the opposite (far-top) corner, appending to the same file.
    python3 measure_scene_box.py corner --out ~/coffee_machine_corners.txt

    # Once the file has exactly 2 recorded corners, compute the box:
    python3 measure_scene_box.py compute --in ~/coffee_machine_corners.txt \\
        --name coffee_machine

Output of `compute` is ready to paste into scene_objects_real.yaml, e.g.:
    coffee_machine.shape_type: "box"
    coffee_machine.pose.x: 0.1234
    coffee_machine.pose.y: 0.5678
    coffee_machine.pose.z: -0.0123
    coffee_machine.pose.yaw: 0.0
    coffee_machine.box_names: ["body"]
    coffee_machine.boxes.body.size: [0.30, 0.40, 0.50]
    coffee_machine.boxes.body.local_pose: [0.0, 0.0, 0.0, 0.0]

yaw is always written as 0.0 — this script only measures an axis-aligned
bounding box (two corners, no rotation capture). If an object is
noticeably rotated relative to base_link's X/Y axes, measure/set yaw by
hand afterward (same convention as countertop/wall's existing entries).

Nodes that must already be running:
    - Real robot driver / joint_state_broadcaster publishing TF
"""

import argparse
import json
import sys

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException

BASE_FRAME = "base_link"
EE_FRAME = "tool0"


def lookup_position(node, tf_buffer, target_frame, source_frame, timeout_sec=5.0):
    deadline = node.get_clock().now() + rclpy.duration.Duration(seconds=timeout_sec)
    while node.get_clock().now() < deadline:
        try:
            tf = tf_buffer.lookup_transform(source_frame, target_frame, Time())
            t = tf.transform.translation
            return (t.x, t.y, t.z)
        except TransformException:
            rclpy.spin_once(node, timeout_sec=0.2)
    raise RuntimeError(
        f"Could not look up {source_frame} -> {target_frame} within {timeout_sec}s "
        "(is the robot driver / TF publisher running?)"
    )


def cmd_corner(args):
    rclpy.init()
    node = Node("measure_scene_box_corner")
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)

    try:
        x, y, z = lookup_position(node, tf_buffer, EE_FRAME, BASE_FRAME, args.timeout)
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)

    print(f"Recorded corner: xyz=({x:.4f}, {y:.4f}, {z:.4f})")
    with open(args.out, "a") as f:
        f.write(json.dumps({"x": x, "y": y, "z": z}) + "\n")
    print(f"Appended to {args.out}")

    rclpy.shutdown()


def cmd_compute(args):
    with open(args.in_file) as f:
        lines = [line.strip() for line in f if line.strip()]

    if len(lines) != 2:
        print(
            f"ERROR: expected exactly 2 recorded corners in {args.in_file}, "
            f"found {len(lines)}. Run `corner` twice (once per opposite "
            "corner) before `compute`.",
            file=sys.stderr,
        )
        sys.exit(1)

    c1 = json.loads(lines[0])
    c2 = json.loads(lines[1])

    center = {axis: (c1[axis] + c2[axis]) / 2.0 for axis in ("x", "y", "z")}
    size = {axis: abs(c1[axis] - c2[axis]) for axis in ("x", "y", "z")}

    name = args.name
    lines_out = [
        f'{name}.shape_type: "box"',
        f"{name}.pose.x: {center['x']:.4f}",
        f"{name}.pose.y: {center['y']:.4f}",
        f"{name}.pose.z: {center['z']:.4f}",
        f"{name}.pose.yaw: 0.0",
        "",
        f'{name}.box_names: ["body"]',
        f"{name}.boxes.body.size: [{size['x']:.4f}, {size['y']:.4f}, {size['z']:.4f}]",
        f"{name}.boxes.body.local_pose: [0.0, 0.0, 0.0, 0.0]",
    ]
    text = "\n".join(lines_out)
    print(text)
    print(
        "\nNote: sizes below ~0.02m on any axis usually mean the two corners "
        "were on the same face, not opposite corners — re-measure if so.",
        file=sys.stderr,
    )


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    p_corner = sub.add_parser("corner", help="Record the current tool0 position as one corner")
    p_corner.add_argument("--out", required=True, help="Text file to append this corner to")
    p_corner.add_argument("--timeout", type=float, default=5.0, help="TF lookup timeout (s)")
    p_corner.set_defaults(func=cmd_corner)

    p_compute = sub.add_parser(
        "compute", help="Compute box center+size from 2 recorded corners")
    p_compute.add_argument("--in", dest="in_file", required=True,
                            help="Text file with exactly 2 recorded corners")
    p_compute.add_argument("--name", required=True,
                            help="Object name prefix for the YAML output (e.g. coffee_machine)")
    p_compute.set_defaults(func=cmd_compute)

    return parser


if __name__ == "__main__":
    args = build_parser().parse_args()
    args.func(args)