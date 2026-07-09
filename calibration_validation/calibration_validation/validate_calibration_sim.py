#!/usr/bin/env python3
"""Sim-only accuracy check: compares calibration_broadcaster_node's
broadcast TF against simulation's own ground-truth TF for the same
physical camera frame.

Why this only works in sim: the ground-truth frame comes from the URDF
(robot_state_publisher already knows exactly where the camera is bolted,
since it's a fixed joint) — there is no such independent second source of
truth on the real robot, which is the entire reason calibration exists
there.

Run standalone once calibration_broadcaster_node has broadcast its result:
    python3 validate_calibration_sim.py
"""

import math

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException
from tf_transformations import euler_from_quaternion

KNOWN_CHAIN_FRAME = "base_link"
# The URDF-declared frame sim publishes as ground truth (this is 
# already correct relative to base_link via robot_state_publisher, 
# no manual offset math needed).
GROUND_TRUTH_FRAME = "wrist_rgbd_camera_depth_optical_frame"
# calibration_broadcaster_node's broadcast_frame_suffix (see
# calibration_broadcaster_sim.yaml) appended to the same base name.
CALIBRATED_FRAME = GROUND_TRUTH_FRAME + "_calibrated"

# Verdict bands. Position is against ground truth (not self-consistency
# spread, which calibration_process.md's mean/max_spread_deg already
# cover) — thresholds are a starting point for sim's scale (UR3e ~0.5m
# reach; calibration is measuring cm-scale offsets), not rigorously
# derived. Retune once real numbers are seen. Orientation reuses the same
# band shape as calibration_process.md's spread bands, since both are
# angular-error-in-degrees measurements.
POSITION_GOOD_M = 0.01
POSITION_CHECK_M = 0.03
ORIENTATION_GOOD_DEG = 5.0
ORIENTATION_CHECK_DEG = 15.0


def verdict(value, good_threshold, check_threshold):
    if value < good_threshold:
        return "GOOD"
    if value < check_threshold:
        return "CHECK"
    return "BAD"


def quaternion_angle_deg(q_a, q_b):
    """Angular difference (degrees) between two quaternions (x,y,z,w
    tuples), accounting for the q/-q double-cover of SO(3) — same
    approach as orientation_averaging.cpp's angularDeviationDeg."""
    dot = abs(sum(a * b for a, b in zip(q_a, q_b)))
    dot = min(1.0, max(-1.0, dot))
    return 2.0 * math.acos(dot) * 180.0 / math.pi


class ValidateCalibrationSim(Node):

    def __init__(self):
        super().__init__("validate_calibration_sim")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.check_once)
        self.done = False

    def check_once(self):
        if self.done:
            return

        try:
            ground_truth = self.tf_buffer.lookup_transform(
                KNOWN_CHAIN_FRAME, GROUND_TRUTH_FRAME, rclpy.time.Time())
            calibrated = self.tf_buffer.lookup_transform(
                KNOWN_CHAIN_FRAME, CALIBRATED_FRAME, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(
                f"Waiting for both '{GROUND_TRUTH_FRAME}' and '{CALIBRATED_FRAME}' "
                f"to be available: {ex}")
            return

        self.done = True

        gt_t = ground_truth.transform.translation
        cal_t = calibrated.transform.translation
        position_error_m = math.sqrt(
            (gt_t.x - cal_t.x) ** 2 + (gt_t.y - cal_t.y) ** 2 + (gt_t.z - cal_t.z) ** 2)

        gt_q = ground_truth.transform.rotation
        cal_q = calibrated.transform.rotation
        orientation_error_deg = quaternion_angle_deg(
            (gt_q.x, gt_q.y, gt_q.z, gt_q.w), (cal_q.x, cal_q.y, cal_q.z, cal_q.w))

        position_verdict = verdict(position_error_m, POSITION_GOOD_M, POSITION_CHECK_M)
        orientation_verdict = verdict(
            orientation_error_deg, ORIENTATION_GOOD_DEG, ORIENTATION_CHECK_DEG)

        self.get_logger().info(
            "\n"
            f"  Ground truth : '{KNOWN_CHAIN_FRAME}' -> '{GROUND_TRUTH_FRAME}'\n"
            f"  Calibrated   : '{KNOWN_CHAIN_FRAME}' -> '{CALIBRATED_FRAME}'\n"
            f"  Position error:    {position_error_m * 100:6.2f} cm   [{position_verdict}]\n"
            f"  Orientation error: {orientation_error_deg:6.2f} deg  [{orientation_verdict}]\n"
        )

        if position_verdict == "BAD" or orientation_verdict == "BAD":
            self.get_logger().warn(
                "Calibration error is outside the expected range — re-check the "
                "settle-sync logic, marker distance/lighting, or re-run calibration.")


def main():
    rclpy.init()
    node = ValidateCalibrationSim()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()