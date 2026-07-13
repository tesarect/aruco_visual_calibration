#!/usr/bin/env python3
"""Real-robot quick tool: from the arm's current pose (place it so the
ArUco marker is visible and roughly centered first, e.g. via
pose_capture.py standoff), step the end effector outward along +X, -X, +Y,
-Y, +Z, -Z in turn, watching /aruco_perception/marker_pose after each step.
For each direction, records the distance traveled from the starting pose
to the last step where the marker was still detected, right before the
sweep stopped. The six distances describe a usable area-of-interest box
around the start pose — feed its half-widths into random_pose_capture.py's
--cube-size (and center it on the start pose) so generated random poses
stay within the region the camera can actually see the marker in AND the
arm can actually reach.

Detection is read off /aruco_perception/marker_pose: aruco_detector_node
only publishes on that topic when the configured marker_id is seen in the
current frame (see aruco_detector_node.cpp) — so "no message within
--detect-timeout seconds after a move" is treated as "not detected".

A sweep in a given direction can stop for two DIFFERENT reasons, and the
report labels which one happened — conflating them silently would report
an arm-reachability limit as if it were a camera FOV edge:
  - "fov"   — the move succeeded but the marker was no longer detected.
              This is the boundary you actually want for --cube-size.
  - "reach" — ~/trace_path itself failed (IK/Cartesian-fraction/collision).
              The FOV may extend further than this; the arm just can't get
              there on the current path. By default the step is retried
              once in joint-space (free-space planning, more robust, no
              straight-line guarantee) before being counted as a "reach"
              boundary — pass --no-joint-space-retry to disable that and
              report the very first planning failure as-is. There is no
              YAML/param override for planAndExecuteCartesian's internal
              min_fraction threshold today (it's a hardcoded C++ default in
              trajectory_planner.hpp, not exposed via TracePath.srv or the
              params file) — the joint-space retry is the only way to push
              past a purely Cartesian-path failure from this script.

Usage:
    python3 fov_boundary_sweep.py --step 0.01 --max-steps 30 --out ~/fov_area.txt

Nodes that must already be running:
    - trajectory_planner_node (moves the arm one small step at a time)
    - aruco_detector_node, already pointed at the live camera feed, with
      the marker physically in view at the start pose
"""

import argparse
import datetime
import sys

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from geometry_msgs.msg import Pose, PoseStamped
from visual_calibration_msgs.srv import TracePath

EE_FRAME = "rg2_gripper_aruco_link"
BASE_FRAME = "base_link"
MARKER_POSE_TOPIC = "/aruco_perception/marker_pose"

# (label, unit vector in base_link X/Y/Z)
DIRECTIONS = [
    ("+X", (1.0, 0.0, 0.0)),
    ("-X", (-1.0, 0.0, 0.0)),
    ("+Y", (0.0, 1.0, 0.0)),
    ("-Y", (0.0, -1.0, 0.0)),
    ("+Z", (0.0, 0.0, 1.0)),
    ("-Z", (0.0, 0.0, -1.0)),
]


class FovBoundarySweep(Node):

    def __init__(self, args):
        super().__init__("fov_boundary_sweep")
        self.args = args
        self.last_marker_stamp = None
        self.marker_sub = self.create_subscription(
            PoseStamped, MARKER_POSE_TOPIC, self._marker_callback, 10)
        self.trace_path_client = self.create_client(
            TracePath, "/trajectory_planner/trace_path")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _marker_callback(self, msg):
        self.last_marker_stamp = self.get_clock().now()

    def wait_for_service(self, timeout=10.0):
        if not self.trace_path_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(
                "/trajectory_planner/trace_path not available — "
                "is trajectory_planner_node running?")
            return False
        return True

    def get_current_pose(self):
        deadline = self.get_clock().now() + rclpy.duration.Duration(seconds=5.0)
        while self.get_clock().now() < deadline:
            try:
                tf = self.tf_buffer.lookup_transform(BASE_FRAME, EE_FRAME, Time())
                pose = Pose()
                pose.position.x = tf.transform.translation.x
                pose.position.y = tf.transform.translation.y
                pose.position.z = tf.transform.translation.z
                pose.orientation = tf.transform.rotation
                return pose
            except TransformException:
                rclpy.spin_once(self, timeout_sec=0.2)
        raise RuntimeError(f"Could not look up {BASE_FRAME} -> {EE_FRAME}")

    def move_to(self, pose, joint_space=None):
        """joint_space=None uses --joint-space's default; pass True/False
        to force a specific planning mode for this call (used by the
        Cartesian-failure retry in sweep_direction)."""
        use_joint_space = self.args.joint_space if joint_space is None else joint_space
        request = TracePath.Request()
        request.waypoints = [pose]
        request.planning_mode = (
            TracePath.Request.PLANNING_MODE_JOINT_SPACE
            if use_joint_space else TracePath.Request.PLANNING_MODE_CARTESIAN
        )
        future = self.trace_path_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        result = future.result()
        if result is None:
            self.get_logger().error("~/trace_path call timed out.")
            return False, "~/trace_path call timed out"
        if not result.success:
            return False, result.message
        return True, ""

    def marker_visible(self):
        """True if a marker_pose message has arrived within
        --detect-timeout seconds of now (settle first, then watch)."""
        settle_deadline = self.get_clock().now() + rclpy.duration.Duration(
            seconds=self.args.settle_sec)
        while self.get_clock().now() < settle_deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.last_marker_stamp = None
        watch_deadline = self.get_clock().now() + rclpy.duration.Duration(
            seconds=self.args.detect_timeout)
        while self.get_clock().now() < watch_deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.last_marker_stamp is not None:
                return True
        return False

    def sweep_direction(self, label, unit_xyz, start_pose):
        dx, dy, dz = unit_xyz
        last_good_distance = 0.0
        boundary_reason = "max_steps"
        pose = Pose()
        pose.orientation = start_pose.orientation

        for step_num in range(1, self.args.max_steps + 1):
            distance = step_num * self.args.step
            pose.position.x = start_pose.position.x + dx * distance
            pose.position.y = start_pose.position.y + dy * distance
            pose.position.z = start_pose.position.z + dz * distance

            moved, message = self.move_to(pose)
            if not moved and not self.args.no_joint_space_retry and not self.args.joint_space:
                self.get_logger().warn(
                    f"{label}: Cartesian move failed at {distance:.3f}m "
                    f"({message}) — retrying in joint-space.")
                moved, message = self.move_to(pose, joint_space=True)

            if not moved:
                self.get_logger().warn(
                    f"{label}: move failed at {distance:.3f}m ({message}) — "
                    "reach/IK limit, not a camera FOV edge.")
                boundary_reason = "reach"
                break

            if not self.marker_visible():
                self.get_logger().info(
                    f"{label}: marker lost at {distance:.3f}m "
                    f"(last seen at {last_good_distance:.3f}m)")
                boundary_reason = "fov"
                break

            last_good_distance = distance
            self.get_logger().info(f"{label}: marker visible at {distance:.3f}m")
        else:
            self.get_logger().warn(
                f"{label}: reached --max-steps ({self.args.max_steps}) without "
                "losing the marker or failing to move — area may extend "
                "further; increase --max-steps.")

        return last_good_distance, boundary_reason

    def run(self):
        if not self.wait_for_service():
            return None

        start_pose = self.get_current_pose()
        self.get_logger().info(
            f"Start pose ({BASE_FRAME}-relative): "
            f"xyz=({start_pose.position.x:.4f}, {start_pose.position.y:.4f}, "
            f"{start_pose.position.z:.4f})")

        if not self.marker_visible():
            self.get_logger().error(
                "Marker not visible at the starting pose — place the arm so the "
                "marker is in view before running this sweep.")
            return None

        results = {}
        for label, unit_xyz in DIRECTIONS:
            results[label] = self.sweep_direction(label, unit_xyz, start_pose)
            # Return to start before sweeping the next direction so each
            # direction is measured independently from the same origin.
            self.move_to(start_pose)

        return start_pose, results


def write_report(args, start_pose, results):
    # results[label] = (distance, reason) where reason is "fov", "reach",
    # or "max_steps". Only "fov" boundaries are true camera-visibility
    # limits — a "reach" boundary means the FOV might extend further than
    # reported, the arm just couldn't get there on this path.
    half_x = min(results["+X"][0], results["-X"][0])
    half_y = min(results["+Y"][0], results["-Y"][0])
    half_z = min(results["+Z"][0], results["-Z"][0])
    any_reach_limited = any(reason == "reach" for _, reason in results.values())

    stamp = datetime.datetime.now().isoformat(timespec="seconds")
    lines = [
        f"[{stamp}] FOV boundary sweep from {BASE_FRAME}-relative start pose "
        f"xyz=({start_pose.position.x:.4f}, {start_pose.position.y:.4f}, "
        f"{start_pose.position.z:.4f})",
    ]
    for label in ("+X", "-X", "+Y", "-Y", "+Z", "-Z"):
        distance, reason = results[label]
        lines.append(f"  {label}: {distance:.4f} m  (boundary: {reason})")
    lines.append(
        f"  Symmetric area-of-interest half-widths (min of each axis pair): "
        f"x={half_x:.4f} m, y={half_y:.4f} m, z={half_z:.4f} m")
    if any_reach_limited:
        lines.append(
            "  NOTE: at least one direction stopped on a 'reach' boundary "
            "(arm/IK limit), not a camera FOV edge — the true FOV may "
            "extend further there. The half-width above is still a safe "
            "--cube-size bound (random poses will stay reachable), but is "
            "NOT necessarily the camera's actual visibility limit on that "
            "axis.")
    lines.append(
        f"  Suggested random_pose_capture.py args: "
        f"--center-x {start_pose.position.x:.4f} --center-y {start_pose.position.y:.4f} "
        f"--center-z {start_pose.position.z:.4f} "
        f"--cube-size {2 * min(half_x, half_y, half_z):.4f}")
    lines.append("")

    text = "\n".join(lines)
    print(text)
    if args.out:
        with open(args.out, "a") as f:
            f.write(text)
        print(f"Appended to {args.out}")


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--step", type=float, default=0.01,
                         help="Step size per move along each axis (m)")
    parser.add_argument("--max-steps", type=int, default=30,
                         help="Max steps to take per direction before giving up")
    parser.add_argument("--settle-sec", type=float, default=0.5,
                         help="Pause after each move before checking detection")
    parser.add_argument("--detect-timeout", type=float, default=2.0,
                         help="Seconds to wait for a marker_pose message before "
                              "declaring the marker not detected")
    parser.add_argument("--joint-space", action="store_true",
                         help="Use joint-space planning instead of Cartesian for every move")
    parser.add_argument("--no-joint-space-retry", action="store_true",
                         help="Don't retry a failed Cartesian move in joint-space before "
                              "counting it as a 'reach' boundary (default: retry once)")
    parser.add_argument("--out", default=None, help="Optional text file to append the report to")
    return parser


def main():
    args = build_parser().parse_args()
    rclpy.init()
    node = FovBoundarySweep(args)
    try:
        result = node.run()
        if result is not None:
            start_pose, results = result
            write_report(args, start_pose, results)
        else:
            sys.exit(1)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
