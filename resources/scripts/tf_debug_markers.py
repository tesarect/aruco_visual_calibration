#!/usr/bin/env python3
"""Publishes RViz axis markers at key TF frames plus the computed
offsetInFrontOf() target pose, so the standoff/orientation math used by
trajectory_planner can be checked visually before running plan+execute.

Run standalone alongside the sim:
    python3 tf_debug_markers.py

Each watched frame (and the computed target) publishes on its own topic
under /tf_debug_markers/<frame_name>, so each can be added as a separate
MarkerArray display in RViz and toggled on/off independently.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from tf_transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, TransformStamped

PLANNING_FRAME = "base_link"
CAMERA_FRAME = "wrist_rgbd_camera_depth_optical_frame"
WATCH_FRAMES = [
    "base_link",
    "wrist_rgbd_camera_depth_optical_frame",
    "tool0",
    "rg2_gripper_base_link",
    "rg2_gripper_aruco_link",
    "rg2_gripper_left_thumb",
    "rg2_gripper_right_thumb",
]

# Kept in sync with trajectory_planner_sim.yaml's standoff_m/facing_rpy_rad.
STANDOFF_M = 0.25
FACING_RPY_RAD = (3.14159265, 0.0, 1.57079633)
AXIS_LENGTH = 0.3
AXIS_DIAMETER = 0.009


def quat_to_rpy_deg(q):
    x, y, z, w = q
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_rotate(q, v):
    qx, qy, qz, qw = q
    vx, vy, vz = v
    uvx = qy * vz - qz * vy
    uvy = qz * vx - qx * vz
    uvz = qx * vy - qy * vx
    uuvx = qy * uvz - qz * uvy
    uuvy = qz * uvx - qx * uvz
    uuvz = qx * uvy - qy * uvx
    return (
        vx + 2.0 * (qw * uvx + uuvx),
        vy + 2.0 * (qw * uvy + uuvy),
        vz + 2.0 * (qw * uvz + uuvz),
    )


def offset_in_front_of(camera_tf: TransformStamped, standoff_m: float, facing_rpy_rad) -> Pose:
    """Mirrors trajectory_planner.cpp's offsetInFrontOf(): move standoff_m
    along the camera's local +Z, then rotate by facing_rpy_rad (applied in
    the camera's own local frame) to get the goal orientation — the same
    facing_rpy_rad value used in trajectory_planner_sim.yaml."""
    t = camera_tf.transform.translation
    r = camera_tf.transform.rotation
    cam_pos = (t.x, t.y, t.z)
    cam_quat = (r.x, r.y, r.z, r.w)

    offset_local = (0.0, 0.0, standoff_m)
    offset_world = quat_rotate(cam_quat, offset_local)
    result_pos = (
        cam_pos[0] + offset_world[0],
        cam_pos[1] + offset_world[1],
        cam_pos[2] + offset_world[2],
    )

    facing_quat = quaternion_from_euler(*facing_rpy_rad)
    result_quat = quat_mul(cam_quat, facing_quat)

    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = result_pos
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = result_quat
    return pose


class TfDebugMarkers(Node):

    def __init__(self):
        super().__init__("tf_debug_markers")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pubs = {
            name: self.create_publisher(MarkerArray, f"/tf_debug_markers/{name}", 10)
            for name in WATCH_FRAMES + ["computed_target"]
        }
        self.timer = self.create_timer(0.5, self.on_timer)

    def axis_markers(self, ns: str, marker_id_base: int, pose: Pose, frame_id: str):
        colors = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]
        local_axes = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]
        q = (
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
        )
        markers = []
        for i, (axis, color) in enumerate(zip(local_axes, colors)):
            world_axis = quat_rotate(q, axis)
            end = Pose()
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = ns
            m.id = marker_id_base + i
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.points = [
                _point(pose.position.x, pose.position.y, pose.position.z),
                _point(
                    pose.position.x + world_axis[0] * AXIS_LENGTH,
                    pose.position.y + world_axis[1] * AXIS_LENGTH,
                    pose.position.z + world_axis[2] * AXIS_LENGTH,
                ),
            ]
            m.scale.x = AXIS_DIAMETER
            m.scale.y = AXIS_DIAMETER * 2.0
            m.scale.z = 0.0
            m.color.r, m.color.g, m.color.b = color
            m.color.a = 1.0
            markers.append(m)
        return markers

    def text_marker(self, ns: str, marker_id: int, pose: Pose, frame_id: str):
        q = (
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
        )
        roll, pitch, yaw = quat_to_rpy_deg(q)
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns + "_text"
        m.id = marker_id
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = pose.position.x
        m.pose.position.y = pose.position.y
        m.pose.position.z = pose.position.z + 0.05
        m.pose.orientation.w = 1.0
        m.scale.z = 0.06
        m.color.r = m.color.g = m.color.b = 1.0
        m.color.a = 1.0
        m.text = (
            f"{ns}\n"
            f"xyz=[{pose.position.x:.3f},{pose.position.y:.3f},{pose.position.z:.3f}]\n"
            f"rpy(deg)=[{roll:.1f},{pitch:.1f},{yaw:.1f}]"
        )
        return m

    def on_timer(self):
        for frame in WATCH_FRAMES:
            try:
                tf = self.tf_buffer.lookup_transform(
                    PLANNING_FRAME, frame, Time())
            except TransformException as ex:
                self.get_logger().warn(f"no TF for {frame}: {ex}", throttle_duration_sec=5.0)
                continue
            pose = Pose()
            pose.position.x = tf.transform.translation.x
            pose.position.y = tf.transform.translation.y
            pose.position.z = tf.transform.translation.z
            pose.orientation = tf.transform.rotation
            out = MarkerArray()
            out.markers.extend(self.axis_markers(frame, 0, pose, PLANNING_FRAME))
            out.markers.append(self.text_marker(frame, 3, pose, PLANNING_FRAME))
            self.pubs[frame].publish(out)

        try:
            camera_tf = self.tf_buffer.lookup_transform(
                PLANNING_FRAME, CAMERA_FRAME, Time())
            target_pose = offset_in_front_of(camera_tf, STANDOFF_M, FACING_RPY_RAD)
            out = MarkerArray()
            out.markers.extend(
                self.axis_markers("computed_target", 0, target_pose, PLANNING_FRAME))
            out.markers.append(
                self.text_marker("computed_target", 3, target_pose, PLANNING_FRAME))
            self.pubs["computed_target"].publish(out)
        except TransformException as ex:
            self.get_logger().warn(f"no TF for target computation: {ex}", throttle_duration_sec=5.0)


def _point(x, y, z):
    from geometry_msgs.msg import Point
    p = Point()
    p.x, p.y, p.z = x, y, z
    return p


def main():
    rclpy.init()
    node = TfDebugMarkers()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()