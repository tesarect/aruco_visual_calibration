#!/usr/bin/env python3
"""Real-robot quick tool: generate random end-effector poses inside a cubic
volume in front of the camera (rg2_gripper_aruco_link kept roughly facing
the camera at every pose), moving there via trajectory_planner's
~/trace_path, and saving one RGB frame per pose — for building a YOLO
training set of the ArUco marker at varied positions/distances.

The cubic volume is defined relative to a CENTER pose you supply (the
standoff pose you jogged/moved to first, e.g. via pose_capture.py standoff
— point this script at that same base_link-relative xyz). Each random
sample offsets X/Y/Z independently within +-half_size of that center,
including Z (distance from camera), as requested. Orientation is kept
fixed at the center pose's orientation (facing the camera) — only position
varies, matching how trajectory_planner already treats its own polygon
waypoints (position varies, facing_rpy_rad fixed).

Usage:
    python3 random_pose_capture.py \
        --center-x 0.0 --center-y 0.0 --center-z 0.3 \
        --roll 3.14159265 --pitch 0.0 --yaw 0.0 \
        --cube-size 0.15 --count 40 --out ~/yolo_captures \
        --env real

Nodes that must already be running:
    - trajectory_planner_node (moves the arm to each sampled pose)
    - Camera driver publishing the RGB topic (real: /D415/color/image_raw,
      sim: /wrist_rgbd_depth_sensor/image_raw)
    - (real robot) Zenoh bridge if capturing over Zenoh
"""

import argparse
import os
import random
import sys
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from tf_transformations import quaternion_from_euler
from visual_calibration_msgs.srv import TracePath

RGB_TOPICS = {
    "sim": "/wrist_rgbd_depth_sensor/image_raw",
    "real": "/D415/color/image_raw",
}


class RandomPoseCapture(Node):

    def __init__(self, args):
        super().__init__("random_pose_capture")
        self.args = args
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_sub = self.create_subscription(
            Image, RGB_TOPICS[args.env], self._image_callback, 10)
        self.trace_path_client = self.create_client(
            TracePath, "/trajectory_planner/trace_path")

        base_quat = quaternion_from_euler(args.roll, args.pitch, args.yaw)
        self.base_orientation = base_quat

        os.makedirs(args.out, exist_ok=True)

    def _image_callback(self, msg):
        self.latest_image = msg

    def wait_for_service(self, timeout=10.0):
        if not self.trace_path_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(
                "/trajectory_planner/trace_path not available — "
                "is trajectory_planner_node running?")
            return False
        return True

    def random_pose(self):
        half = self.args.cube_size / 2.0
        pose = Pose()
        pose.position.x = self.args.center_x + random.uniform(-half, half)
        pose.position.y = self.args.center_y + random.uniform(-half, half)
        pose.position.z = self.args.center_z + random.uniform(-half, half)
        pose.orientation.x = self.base_orientation[0]
        pose.orientation.y = self.base_orientation[1]
        pose.orientation.z = self.base_orientation[2]
        pose.orientation.w = self.base_orientation[3]
        return pose

    def move_to(self, pose):
        request = TracePath.Request()
        request.waypoints = [pose]
        request.planning_mode = (
            TracePath.Request.PLANNING_MODE_JOINT_SPACE
            if self.args.joint_space else TracePath.Request.PLANNING_MODE_CARTESIAN
        )
        future = self.trace_path_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        result = future.result()
        if result is None:
            self.get_logger().error("~/trace_path call timed out.")
            return False
        if not result.success:
            self.get_logger().warn(f"~/trace_path failed: {result.message}")
            return False
        return True

    def capture_frame(self, index, pose):
        self.latest_image = None
        deadline = self.get_clock().now() + rclpy.duration.Duration(
            seconds=self.args.settle_sec + self.args.capture_timeout)
        settle_deadline = self.get_clock().now() + rclpy.duration.Duration(
            seconds=self.args.settle_sec)
        while self.get_clock().now() < settle_deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        while self.latest_image is None and self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.latest_image is None:
            self.get_logger().warn(f"Sample {index}: no image received — skipping save.")
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding="bgr8")
        filename = os.path.join(self.args.out, f"sample_{index:04d}.png")
        cv2.imwrite(filename, cv_image)
        self.get_logger().info(
            f"Sample {index}: saved {filename} "
            f"(pose xyz=({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}))")
        return True

    def run(self):
        if not self.wait_for_service():
            return
        saved = 0
        for i in range(self.args.count):
            pose = self.random_pose()
            if not self.move_to(pose):
                continue
            if self.capture_frame(i, pose):
                saved += 1
        self.get_logger().info(f"Done: {saved}/{self.args.count} samples saved to {self.args.out}")


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--env", choices=["sim", "real"], default="real")
    parser.add_argument("--center-x", type=float, default=0.0,
                         help="Cube center X, base_link-relative (m)")
    parser.add_argument("--center-y", type=float, default=0.0,
                         help="Cube center Y, base_link-relative (m)")
    parser.add_argument("--center-z", type=float, default=0.3,
                         help="Cube center Z, base_link-relative (m)")
    parser.add_argument("--cube-size", type=float, default=0.15,
                         help="Full side length of the sampling cube (m)")
    parser.add_argument("--roll", type=float, default=3.14159265, help="Fixed orientation roll (rad)")
    parser.add_argument("--pitch", type=float, default=0.0, help="Fixed orientation pitch (rad)")
    parser.add_argument("--yaw", type=float, default=0.0, help="Fixed orientation yaw (rad)")
    parser.add_argument("--count", type=int, default=20, help="Number of random poses to sample")
    parser.add_argument("--out", default="./yolo_captures", help="Output directory for images")
    parser.add_argument("--settle-sec", type=float, default=1.0,
                         help="Pause after each move before capturing (let the arm/image settle)")
    parser.add_argument("--capture-timeout", type=float, default=3.0,
                         help="Max seconds to wait for a fresh image after settling")
    parser.add_argument("--joint-space", action="store_true",
                         help="Use joint-space planning instead of Cartesian")
    return parser


def main():
    args = build_parser().parse_args()
    rclpy.init()
    node = RandomPoseCapture(args)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
