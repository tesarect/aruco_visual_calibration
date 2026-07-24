#!/usr/bin/env python3
"""YOLO-backed drop-in alternative to aruco_perception's classical
aruco_detector_node (see aruco_perception/src/aruco_detector/aruco_detector_node.cpp).

This node is a normal rclpy node -- it imports cv_bridge/rclpy only, NEVER
ultralytics. It calls YOLO-pipeline/inference_server.py (running inside the
isolated ~/yolo_venv, a completely separate process/venv) over plain HTTP on
localhost. This keeps ROS's system OpenCV 4.5.4 (that cv_bridge/
image_transport are compiled against) and ultralytics' bundled newer OpenCV
in two separate processes at all times, per this project's locked
ABI-isolation architecture.

What it does, per image callback:
  1. Convert sensor_msgs/Image -> OpenCV BGR array via cv_bridge (wrapped in
     try/except -- a bad conversion logs and skips the frame, matching
     aruco_detector_node.cpp's own error-mitigation pattern, never crashes
     the node).
  2. JPEG-encode + base64-encode the frame.
  3. Build the /detect request body using the LATEST received camera_info
     message's k/d fields -- intrinsics are never hardcoded/cached beyond
     "the most recent camera_info message" (see inference_server.py's
     contract: camera_matrix/dist_coeffs are required on every request, no
     server-side default).
  4. POST to the inference server (localhost, short timeout so a hung
     server can't stall the ROS executor -- logged and skipped, not fatal).
  5. If "aruco_marker" is present in the response AND the "active"
     parameter is true: convert its Rodrigues rvec to a quaternion and
     publish geometry_msgs/PoseStamped on pose_topic (default
     /aruco_perception/marker_pose) -- same topic, message type, and
     frame_id convention (the incoming Image message's own header) as the
     classical aruco_detector_node, so calibration_broadcaster_node needs
     zero changes regardless of which detector produced the pose.

     classical/hybrid switch: this node and aruco_perception's classical
     ArucoDetectorNode both publish PoseStamped on the SAME pose_topic --
     exactly one should be "active" (actually publishing) at a time. Unlike
     the classical node (which skips detection entirely when inactive,
     since it has nothing else to do), THIS node still runs YOLO detection
     and still publishes cup_holder/hole detections_2d every frame
     regardless of "active" -- only the aruco_marker PoseStamped publish is
     gated. Default false (classical is active by default; see
     aruco_detector_node.hpp). Flipped live by
     calibration_orchestrator_node via the standard ROS set_parameters
     service, re-read fresh (never cached) on every frame, so a switch
     takes effect on the very next frame with no restart.
  6. Publishes ONE visual_calibration_msgs/Detection2DArray on
     detections_2d_topic (default /aruco_perception/detections_2d), every
     frame, containing whichever of "aruco_marker"/"cup_holder"/"hole"
     were present in the response:
       - "aruco_marker" (added 2026-07-23): cx/cy = average of the 4
         returned corners, confidence 1.0, bbox = corner min/max -- same
         convention as aruco_detector_node.cpp's classical publish, so
         calibration_orchestrator_node's image-based centering
         (centerOnMarkerUsingImage) works identically regardless of which
         detector is active. Published UNCONDITIONALLY (like cup_holder/
         hole below), not gated by "active" -- an operator can watch this
         detector's own view of the marker even while classical is the
         one actually driving calibration.
       - "cup_holder"/"hole": the intended consumer is depth-perception's
         own hole/cupholder 3D pose pipeline (it looks up depth at each
         detection's cx/cy, using bbox to sample a small neighborhood
         rather than one noisy pixel, then back-projects to 3D itself;
         single-frame results are noisy, so it filters/votes across
         multiple frames on its own side -- see that message type's own
         header comment for the full rationale).
     NOT vision_msgs/Detection2DArray -- a project-local custom type
     instead, see visual_calibration_msgs/msg/Detection2DArray.msg for why.

Request/response JSON contract with inference_server.py (kept in sync with
YOLO-pipeline/README.md's "How this package talks to visual_calibration --
API structure" section -- that section is the ground truth; this comment is
a summary for readers of this file only):

  POST http://<inference_server_url>/detect
  Request body:
    {
      "image_jpeg_base64": "<base64 JPEG bytes>",
      "camera_matrix": [[fx,0,cx],[0,fy,cy],[0,0,1]],
      "dist_coeffs": [d0, d1, ...],
      "conf": 0.25
    }
  Response body (each key entirely OMITTED, never null/empty, if that class
  was not found in the frame):
    {
      "aruco_marker": {"rvec": [rx,ry,rz], "tvec": [tx,ty,tz],
                        "corners": [[x,y],[x,y],[x,y],[x,y]]},
      "cup_holder": [{"cx": .., "cy": .., "confidence": .., "bbox": [x1,y1,x2,y2]}, ...],
      "hole": [{"cx": .., "cy": .., "confidence": .., "bbox": [x1,y1,x2,y2]}, ...]
    }
  rvec/tvec: Rodrigues rotation vector / position in meters, camera's
  optical frame. corners: the marker's 4 detected corners, already
  converted back to FULL-FRAME pixel coordinates server-side (see
  aruco_pose.py's corners_to_full_frame -- undoes both the YOLO crop offset
  and any preprocessing-variant resize, e.g. the cascade's upscale_4x
  variant, so these are directly usable against the original image, no
  further transform needed here). cx/cy/bbox: pixel coordinates (2D only,
  no cup_holder/hole 3D pose from this node/server -- no known real-world
  circle size to solve against; depth-perception adds the 3D piece
  downstream). HTTP 400 with {"error": "..."} on a malformed/missing
  request field.
"""

import base64

import cv2
import numpy as np
import requests

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from visual_calibration_msgs.msg import Detection2D, Detection2DArray


def rotation_matrix_to_quaternion(rotation_matrix):
    """Standard rotation-matrix -> quaternion (x, y, z, w) conversion
    (Shepperd's method / the common "largest diagonal element" branch used
    by tf2's own Matrix3x3::getRotation, reimplemented here in plain numpy).

    Kept as a small self-contained function rather than adding a new
    dependency: this project has no rclpy-side tf_transformations usage in
    aruco_perception, and the one place tf_transformations IS already a
    dependency (calibration_validation) only imports
    euler_from_quaternion, not any rotation-matrix conversion -- so there's
    no existing rotation-matrix->quaternion helper to reuse, and pulling in
    scipy for a single well-known ~15 line formula isn't worth a new
    dependency this close to the deadline.
    """
    m = rotation_matrix
    trace = m[0, 0] + m[1, 1] + m[2, 2]

    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m[2, 1] - m[1, 2]) * s
        y = (m[0, 2] - m[2, 0]) * s
        z = (m[1, 0] - m[0, 1]) * s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s

    return (x, y, z, w)


class YoloMarkerBridgeNode(Node):
    """Subscribes to the camera image/camera_info topics, calls the YOLO
    inference server over HTTP, and republishes an aruco_marker detection
    as geometry_msgs/PoseStamped -- a drop-in alternative to
    aruco_detector_node's pose output, for the classical-vs-YOLO detector
    swap described in the top-level todo.txt.
    """

    def __init__(self):
        super().__init__(
            "yolo_marker_bridge_node",
            automatically_declare_parameters_from_overrides=True,
        )

        # NOTE (2026-07-24, fixed a live crash): do NOT explicitly
        # declare_parameter() any of these here -- automatically_declare_
        # parameters_from_overrides=True above already auto-declares every
        # parameter present in the --params-file yaml (yolo_marker_bridge_
        # {sim,real}.yaml set every one of these), so an explicit
        # declare_parameter() call for the same name throws
        # rclpy.exceptions.ParameterAlreadyDeclaredException at
        # construction time -- confirmed live, this crashed the node
        # outright the first time it was actually run. Matches
        # aruco_detector_node.cpp's own pattern (same auto-declare flag,
        # zero explicit declare_parameter calls, only get_parameter reads)
        # -- this file just hadn't been fixed to match it yet. If a NEW
        # parameter not already in both yaml files is ever added here, it
        # WILL need an explicit declare_parameter (yaml won't auto-declare
        # what it doesn't mention) -- just keep it out of this list if it's
        # already yaml-declared.
        # classical/hybrid switch -- default false (classical
        # aruco_detector_node is active by default). Re-read live via
        # get_parameter in image_callback, never cached, so
        # calibration_orchestrator_node's set_parameters call takes effect
        # on the very next frame. Only gates the aruco_marker PoseStamped
        # publish -- cup_holder/hole detections_2d publish unconditionally
        # regardless of this switch (see class doc comment).
        # Overlay -- same convention/param names as aruco_detector_node's
        # ArucoDetectorConfig (publish_overlay_image, overlay_image_topic,
        # overlay_border_color_bgr) and marker_length_m, so both detectors'
        # config files read the same. Gated separately from "active" -- an
        # operator debugging hybrid mode still wants the overlay even if,
        # for some reason, this node's marker_pose publish were disabled.

        self.image_topic = self.get_parameter("image_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.pose_topic = self.get_parameter("pose_topic").value
        self.detections_2d_topic = self.get_parameter(
            "detections_2d_topic"
        ).value
        self.publish_overlay_image = bool(
            self.get_parameter("publish_overlay_image").value
        )
        self.overlay_image_topic = self.get_parameter(
            "overlay_image_topic"
        ).value
        self.overlay_border_color_bgr = tuple(
            int(v) for v in self.get_parameter("overlay_border_color_bgr").value
        )
        self.marker_length_m = float(
            self.get_parameter("marker_length_m").value
        )
        self.inference_server_url = self.get_parameter(
            "inference_server_url"
        ).value
        self.request_timeout_sec = float(
            self.get_parameter("request_timeout_sec").value
        )
        self.confidence_threshold = float(
            self.get_parameter("confidence_threshold").value
        )
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)

        self.bridge = CvBridge()

        # Camera intrinsics: only ever the most recently received
        # camera_info message -- never hardcoded/cached beyond that, per
        # inference_server.py's contract (camera_matrix/dist_coeffs
        # required fresh on every request).
        self.camera_matrix = None
        self.dist_coeffs = None

        self.pose_pub = self.create_publisher(
            PoseStamped, self.pose_topic, 10
        )
        # Same QoS (plain reliable, depth-10 queue) as pose_pub -- a
        # derived detection stream, not raw sensor data, and not a latched
        # "current state" topic (see Detection2DArray.msg's own header
        # comment for the full reasoning depth-perception requested).
        self.detections_2d_pub = self.create_publisher(
            Detection2DArray, self.detections_2d_topic, 10
        )
        if self.publish_overlay_image:
            self.overlay_image_pub = self.create_publisher(
                Image, self.overlay_image_topic, 10
            )
        else:
            self.overlay_image_pub = None

        # Own callback group (2026-07-24, fixed a live bug), separate from
        # the node's default group that ROS's built-in set_parameters
        # service callback runs in -- see main()'s doc comment for the
        # full story (image_callback blocks on a synchronous HTTP request
        # for up to request_timeout_sec, which under a single shared
        # default MutuallyExclusive group also blocks set_parameters from
        # running until that HTTP call returns, even with a
        # MultiThreadedExecutor -- a MultiThreadedExecutor only allows
        # DIFFERENT callback groups to run concurrently, it does not by
        # itself parallelize callbacks that share one group). Both
        # subscriptions share ONE MutuallyExclusiveCallbackGroup (not
        # Reentrant) since image_callback/camera_info_callback don't need
        # to run concurrently WITH EACH OTHER, just concurrently with
        # everything else (parameter service, etc).
        self._sensor_callback_group = MutuallyExclusiveCallbackGroup()
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback,
            qos_profile_sensor_data,
            callback_group=self._sensor_callback_group,
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback,
            qos_profile_sensor_data,
            callback_group=self._sensor_callback_group,
        )

        self.get_logger().info(
            "yolo_marker_bridge_node ready (image_topic: '%s', "
            "camera_info_topic: '%s', pose_topic: '%s', "
            "detections_2d_topic: '%s', inference_server_url: '%s')" % (
                self.image_topic, self.camera_info_topic, self.pose_topic,
                self.detections_2d_topic, self.inference_server_url,
            )
        )

    def camera_info_callback(self, msg):
        # Always refresh (unlike aruco_detector_node.cpp, which latches on
        # first receipt) -- the inference server takes intrinsics fresh on
        # every request rather than caching them server-side, so mirror
        # that "always current" contract on this side too.
        self.camera_matrix = np.array(msg.k, dtype=float).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=float)

    def image_callback(self, msg):
        if self.camera_matrix is None:
            self.get_logger().warn(
                "No camera_info received yet on '%s' -- skipping detection "
                "(need intrinsics for the /detect request)." %
                self.camera_info_topic,
                throttle_duration_sec=5.0,
            )
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(
                "cv_bridge conversion failed: %s" % str(e),
                throttle_duration_sec=5.0,
            )
            return

        ok, jpeg_bytes = cv2.imencode(
            ".jpg", cv_image,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not ok:
            self.get_logger().error(
                "cv2.imencode failed to JPEG-encode the frame -- skipping.",
                throttle_duration_sec=5.0,
            )
            return

        image_b64 = base64.b64encode(jpeg_bytes.tobytes()).decode("ascii")

        request_body = {
            "image_jpeg_base64": image_b64,
            "camera_matrix": self.camera_matrix.tolist(),
            "dist_coeffs": self.dist_coeffs.tolist(),
            "conf": self.confidence_threshold,
        }

        try:
            response = requests.post(
                self.inference_server_url, json=request_body,
                timeout=self.request_timeout_sec,
            )
        except requests.exceptions.RequestException as e:
            self.get_logger().error(
                "Inference server request failed (%s) -- skipping frame. "
                "Is inference_server.py running in ~/yolo_venv?" % str(e),
                throttle_duration_sec=5.0,
            )
            return

        if response.status_code != 200:
            self.get_logger().error(
                "Inference server returned HTTP %d: %s -- skipping frame." %
                (response.status_code, response.text),
                throttle_duration_sec=5.0,
            )
            return

        try:
            result = response.json()
        except ValueError as e:
            self.get_logger().error(
                "Inference server response was not valid JSON: %s" % str(e),
                throttle_duration_sec=5.0,
            )
            return

        # classical/hybrid switch: only publish the marker pose when this
        # node is the active detector -- live re-read, never cached, see
        # class doc comment / the "active" parameter's declare_parameter
        # comment above.
        if "aruco_marker" in result and self.get_parameter("active").value:
            self.publish_marker_pose(msg, result["aruco_marker"])

        # Overlay: independent of "active" -- an operator debugging hybrid
        # mode still wants the visual confirmation even if this node isn't
        # currently the one publishing marker_pose (see publish_overlay_image's
        # declare_parameter comment above).
        if "aruco_marker" in result and self.overlay_image_pub is not None:
            self.publish_overlay_image_msg(msg, cv_image, result["aruco_marker"])

        # cup_holder/hole publish unconditionally, regardless of "active" --
        # depth-perception needs this stream running continuously either way.
        self.publish_detections_2d(msg, result)

    def publish_detections_2d(self, image_msg, result):
        """aruco_marker/cup_holder/hole detections, as
        visual_calibration_msgs/Detection2DArray on detections_2d_topic.
        cup_holder/hole were requested by depth-perception (its own
        hole/cupholder 3D pose pipeline looks up depth at each detection's
        cx/cy, using bbox for a more robust multi-pixel sample).
        aruco_marker was added 2026-07-23 so
        calibration_orchestrator_node's image-based centering
        (centerOnMarkerUsingImage) works identically in hybrid mode as in
        classical mode -- that method only ever reads the marker's pixel
        centroid via this topic's "aruco_marker" class_name entry (see
        ArucoDetectorNode::imageCallback's matching classical-side publish,
        aruco_detector_node.cpp), which this node never emitted before,
        silently making auto-centering fail/time out whenever hybrid mode
        was active (confirmed via calibration_orchestrator_node.hpp's own
        "classical detector only for now" doc comment, now resolved).
        Always publishes, every frame, even with an empty detections[]
        when nothing was found -- a continuous stream consumers filter/vote
        over, not a detected-vs-absent gap they'd need to distinguish from
        "node down".
        """
        array_msg = Detection2DArray()
        array_msg.header = image_msg.header  # same convention as marker_pose

        if "aruco_marker" in result:
            # corners: [[x,y],[x,y],[x,y],[x,y]], already full-frame pixel
            # coordinates (see publish_overlay_image_msg's own comment on
            # this same field). cx/cy/bbox computed identically to
            # aruco_detector_node.cpp's classical publish (average of the
            # 4 corners; confidence 1.0 -- neither detector has a
            # meaningful per-marker confidence score for this class).
            corners = np.array(
                result["aruco_marker"]["corners"], dtype=float
            )
            det = Detection2D()
            det.class_name = "aruco_marker"
            det.cx = float(corners[:, 0].mean())
            det.cy = float(corners[:, 1].mean())
            det.confidence = 1.0
            det.bbox = [
                float(corners[:, 0].min()), float(corners[:, 1].min()),
                float(corners[:, 0].max()), float(corners[:, 1].max()),
            ]
            array_msg.detections.append(det)

        for class_name in ("cup_holder", "hole"):
            for d in result.get(class_name, []):
                det = Detection2D()
                det.class_name = class_name
                det.cx = float(d.get("cx", 0.0))
                det.cy = float(d.get("cy", 0.0))
                det.confidence = float(d.get("confidence", 0.0))
                bbox = d.get("bbox", [0.0, 0.0, 0.0, 0.0])
                det.bbox = [float(v) for v in bbox]
                array_msg.detections.append(det)

        self.detections_2d_pub.publish(array_msg)

    def publish_marker_pose(self, image_msg, marker_result):
        rvec = np.array(marker_result["rvec"], dtype=float)
        tvec = np.array(marker_result["tvec"], dtype=float)

        rotation_matrix, _ = cv2.Rodrigues(rvec)
        qx, qy, qz, qw = rotation_matrix_to_quaternion(rotation_matrix)

        pose_msg = PoseStamped()
        # Same convention as aruco_detector_node.cpp: reuse the incoming
        # Image message's own header (stamp + frame_id) rather than
        # self.get_clock().now() -- matches this project's use_sim_time
        # fix (see progress.md's 2026-07-08 entry / error-mitigation.md
        # #16) and keeps frame_id as the camera's optical frame, exactly
        # as the classical detector does.
        pose_msg.header = image_msg.header
        pose_msg.pose.position.x = float(tvec[0])
        pose_msg.pose.position.y = float(tvec[1])
        pose_msg.pose.position.z = float(tvec[2])
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        self.pose_pub.publish(pose_msg)

    def publish_overlay_image_msg(self, image_msg, cv_image, marker_result):
        """Yellow border + XYZ axes overlay, matching aruco_detector_node.cpp's
        classical overlay_image exactly (same drawDetectedMarkers/
        drawFrameAxes calls, same overlay_border_color_bgr default, same
        bgr8 encoding -- so a viewer/consumer of /aruco_perception/
        overlay_image sees identical visuals regardless of which detector
        produced it). Draws on a COPY of cv_image (never mutates the frame
        used for the /detect request above)."""
        overlay = cv_image.copy()

        # corners: [[x,y],[x,y],[x,y],[x,y]] from inference_server.py,
        # already in full-frame pixel space (see corners_to_full_frame).
        # drawDetectedMarkers expects a list of (1, 4, 2) float32 arrays,
        # one per detected marker -- we only ever have the one.
        corners = np.array(marker_result["corners"], dtype=np.float32).reshape(1, 1, 4, 2)
        cv2.aruco.drawDetectedMarkers(
            overlay, list(corners), None, self.overlay_border_color_bgr
        )

        rvec = np.array(marker_result["rvec"], dtype=np.float64)
        tvec = np.array(marker_result["tvec"], dtype=np.float64)
        cv2.drawFrameAxes(
            overlay, self.camera_matrix, self.dist_coeffs, rvec, tvec,
            self.marker_length_m * 0.5,
        )

        try:
            overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(
                "cv_bridge overlay conversion failed: %s" % str(e),
                throttle_duration_sec=5.0,
            )
            return
        overlay_msg.header = image_msg.header
        self.overlay_image_pub.publish(overlay_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloMarkerBridgeNode()
    # MultiThreadedExecutor, not plain rclpy.spin() (2026-07-24, fixed a
    # live bug) -- image_callback blocks on a synchronous requests.post()
    # to inference_server.py for up to request_timeout_sec (currently 3s,
    # see yolo_marker_bridge_{sim,real}.yaml). Under single-threaded spin,
    # that blocks EVERY other callback on this node too, including the
    # ROS-standard set_parameters service calibration_orchestrator_node's
    # ~/set_detector_mode uses to flip "active" -- confirmed live: that
    # call timed out (orchestrator's own 2s wait, shorter than this node's
    # 3s worst-case block) with "Failed to activate yolo_marker_bridge_node:
    # timed out waiting for response" even though the node was genuinely
    # up and healthy the whole time. A multi-threaded executor lets the
    # parameter-service callback run concurrently on a different thread
    # instead of queueing behind an in-flight HTTP request.
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()