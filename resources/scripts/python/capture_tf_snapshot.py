#!/usr/bin/env python3
"""One-shot diagnostic capture for the hole/cup_holder TF mismatch
investigation (2026-08-05) — grabs everything needed from a SINGLE moment
in time, so there's no cross-time contamination between readings (the arm
doesn't move, the camera calibration doesn't change, between captures).

Captures, in order:
  1. depth_perception_node's own "cup_holder: frame(x=..., y=..., z=...)
     ... radius_px=... patch_half_px=..." diagnostic log line (from
     /rosout) — the camera-frame 3D point + pixel/depth info it actually
     used this moment.
  2. base_link -> D415_color_optical_frame_calibrated (the camera
     calibration TF).
  3. base_link -> cup_holder (the broadcast holder TF).
  4. base_link -> robotiq_85_base_link (for reference/cross-checking
     against a later physical thumb-touch measurement).

Run while depth_perception_node is actively publishing (arm can be
anywhere — does NOT need to be near the holder, does NOT need to move):
    python3 capture_tf_snapshot.py

Prints everything to stdout in one block, ready to paste back for analysis.
"""

import re
import sys
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log
from tf2_ros import Buffer, TransformListener, TransformException


CAMERA_FRAME = "D415_color_optical_frame_calibrated"
KNOWN_FRAME = "base_link"


class SnapshotCapture(Node):
    def __init__(self):
        super().__init__("tf_snapshot_capture")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cup_holder_log_line = None
        self.rosout_sub = self.create_subscription(
            Log, "/rosout", self.rosout_callback, 50
        )

    def rosout_callback(self, msg):
        if self.cup_holder_log_line is not None:
            return
        # depth_perception_node's own log line, one "cup_holder: ..." entry
        # per line inside a multi-line "detections_2d back-projected:" msg.
        if msg.name == "depth_perception_node" and "cup_holder:" in msg.msg:
            for line in msg.msg.split("\n"):
                if "cup_holder:" in line:
                    self.cup_holder_log_line = line.strip()
                    self.get_logger().info(
                        "Captured cup_holder log line: %s" % self.cup_holder_log_line
                    )
                    break

    def try_lookup(self, target, source):
        try:
            tf = self.tf_buffer.lookup_transform(
                target, source, rclpy.time.Time()
            )
            t = tf.transform.translation
            r = tf.transform.rotation
            return {
                "xyz": (t.x, t.y, t.z),
                "quat_xyzw": (r.x, r.y, r.z, r.w),
            }
        except TransformException as ex:
            return {"error": str(ex)}


def main():
    rclpy.init()
    node = SnapshotCapture()

    print("Waiting up to 15s for a fresh depth_perception_node cup_holder log "
          "line on /rosout (arm can be anywhere -- do not move it while this "
          "runs)...", file=sys.stderr)
    deadline = time.time() + 15.0
    while node.cup_holder_log_line is None and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    if node.cup_holder_log_line is None:
        print("WARNING: no cup_holder log line captured in 15s -- is "
              "depth_perception_node running and seeing the cup_holder? "
              "Continuing to capture TFs anyway.", file=sys.stderr)

    # Give tf_buffer a moment to accumulate a few transform messages before
    # looking anything up.
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.1)

    camera_tf = node.try_lookup(KNOWN_FRAME, CAMERA_FRAME)
    cup_holder_tf = node.try_lookup(KNOWN_FRAME, "cup_holder")
    gripper_tf = node.try_lookup(KNOWN_FRAME, "robotiq_85_base_link")

    print("\n" + "=" * 70)
    print("SNAPSHOT (paste everything below back)")
    print("=" * 70)
    print(f"\n[1] depth_perception_node cup_holder log line:")
    print(f"    {node.cup_holder_log_line}")
    print(f"\n[2] {KNOWN_FRAME} -> {CAMERA_FRAME}:")
    print(f"    {camera_tf}")
    print(f"\n[3] {KNOWN_FRAME} -> cup_holder:")
    print(f"    {cup_holder_tf}")
    print(f"\n[4] {KNOWN_FRAME} -> robotiq_85_base_link (arm's current pose, "
          f"for reference):")
    print(f"    {gripper_tf}")
    print("=" * 70)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
