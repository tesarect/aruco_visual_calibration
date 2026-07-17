#!/usr/bin/env python3
"""Quick-and-dirty capture: saves RGB + depth frames from the camera to
disk, one file pair per received RGB frame (or a fixed count).

Sim topics come straight from Gazebo (no extra setup). Real topics come
over the Zenoh bridge — see zenoh-pointcloud/README.md — and require,
before running this script:
    unset CYCLONEDDS_URI
    export ROS_DOMAIN_ID=1
    (Zenoh bridge running: ros2_ws/src/zenoh-pointcloud/init/rosject.sh)

Usage:
    python3 capture_camera.py --env sim                 # save every frame until Ctrl+C
    python3 capture_camera.py --env real --count 5       # save 5 frames then exit
    python3 capture_camera.py --env sim --out ~/captures --every 1.0   # 1 frame/sec
"""

import argparse
import os
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

TOPICS = {
    "sim": {
        "rgb": "/wrist_rgbd_depth_sensor/image_raw",
        "depth": "/wrist_rgbd_depth_sensor/depth/image_raw",
    },
    "real": {
        "rgb": "/D415/color/image_raw",
        "depth": "/D415/aligned_depth_to_color/image_raw",
    },
}


class CameraCapture(Node):

    def __init__(self, env, out_dir, count, every_sec):
        super().__init__("camera_capture")
        self.bridge = CvBridge()
        self.out_dir = out_dir
        self.count = count
        self.every_sec = every_sec
        self.saved = 0
        self.last_save_time = 0.0
        self.latest_depth = None

        rgb_topic = TOPICS[env]["rgb"]
        depth_topic = TOPICS[env]["depth"]

        os.makedirs(out_dir, exist_ok=True)

        self.create_subscription(Image, depth_topic, self.depth_cb, 10)
        self.create_subscription(Image, rgb_topic, self.rgb_cb, 10)

        self.get_logger().info(f"Subscribed to {rgb_topic} and {depth_topic}")
        self.get_logger().info(f"Saving to {out_dir}")

    def depth_cb(self, msg):
        self.latest_depth = msg

    def rgb_cb(self, msg):
        now = time.time()
        if now - self.last_save_time < self.every_sec:
            return

        try:
            # desired_encoding="bgr8" converts regardless of the source
            # encoding (sim publishes rgb8, real publishes bgr8) — see
            # error-mitigation.md #15 for why toCvCopy/imgmsg_to_cv2
            # (a real conversion, not zero-copy) is required here.
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as ex:
            self.get_logger().warn(f"RGB conversion failed: {ex}")
            return

        stamp = int(now * 1000)
        rgb_path = os.path.join(self.out_dir, f"rgb_{stamp}.png")
        cv2.imwrite(rgb_path, rgb)
        self.get_logger().info(f"Saved {rgb_path}")

        if self.latest_depth is not None:
            try:
                depth = self.bridge.imgmsg_to_cv2(
                    self.latest_depth, desired_encoding="passthrough")
                depth_path = os.path.join(self.out_dir, f"depth_{stamp}.png")
                cv2.imwrite(depth_path, depth)
                self.get_logger().info(f"Saved {depth_path}")
            except Exception as ex:
                self.get_logger().warn(f"Depth conversion failed: {ex}")

        self.last_save_time = now
        self.saved += 1

        if self.count is not None and self.saved >= self.count:
            self.get_logger().info(f"Captured {self.saved} frame(s), done.")
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--env", choices=["sim", "real"], default="sim")
    parser.add_argument("--out", default=os.path.expanduser("~/camera_captures"))
    parser.add_argument("--count", type=int, default=None,
                         help="stop after N saved frames (default: run until Ctrl+C)")
    parser.add_argument("--every", type=float, default=0.5,
                         help="minimum seconds between saved frames (default 0.5)")
    args = parser.parse_args()

    rclpy.init()
    node = CameraCapture(args.env, args.out, args.count, args.every)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
