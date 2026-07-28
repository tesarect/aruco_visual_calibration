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
      "conf": 0.25,
      "skip_marker": false
    }
  skip_marker (added 2026-07-27): optional, defaults false server-side if
  omitted. When true, inference_server.py skips the ArUco marker cascade
  entirely for this request (no "aruco_marker" key in the response) --
  cup_holder/hole detection is UNAFFECTED either way, it was never part of
  the cascade. This node decides skip_marker per-frame via
  marker_check_every_n_frames/marker_check_full_rate_when_active (see
  __init__) -- the marker cascade was found live to cost 0.17-0.36s/request
  even when no marker was present, the dominant cost in a ~0.4-0.6s total
  /detect call, starving cup_holder/hole of a smoother detection stream.
  Full-rate checking is always forced (skip_marker=false) whenever this
  node's own "active" param is true, so calibration/auto-centering (both
  of which DO need reliable per-attempt marker detection, unlike
  cup_holder/hole) are never starved by this throttle.
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

  "hole" entries additionally get a hole_number (1-4, fixed image-space
  quadrant: top-left/top-right/bottom-left/bottom-right) on the PUBLISHED
  Detection2D -- NOT part of the inference_server.py response above, this
  is computed client-side in publish_detections_2d()/assign_hole_quadrants()
  since it only needs 2D pixel positions already in the response. See
  Detection2D.msg's hole_number field comment for the full rule.

Stabilized-overlay subscription (2026-07-27) -- a deliberate exception to
this node's otherwise one-directional (image in, poses/detections out)
data flow: also subscribes to depth_perception_node's
visual_calibration_msgs/StablePositionArray (stable_positions_topic,
default /depth_perception/stable_positions) purely to draw its held,
drift-filtered per-instance positions onto the SAME overlay image this
node already publishes, alongside its own raw per-frame YOLO boxes/labels
-- see publish_overlay_image_msg. This exists because a raw per-frame
overlay alone still visibly "flickers" whenever YOLO misses a detection
for a frame or two, even though depth_perception's own held position
never actually gaps. Kept as loosely coupled as that requirement allows:
this node only reads StablePosition's published fields (px/py/drifted/
etc.), never depth_perception's internal rolling-window state, and
degrades gracefully if depth_perception_node isn't running at all (simply
never receives anything on that topic, draws nothing extra).
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
from visual_calibration_msgs.msg import (
    AutoCalibrateStatus,
    Detection2D,
    Detection2DArray,
    StablePositionArray,
)


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


def assign_hole_quadrants(detections, result):
    """Assigns each "hole" Detection2D in `detections` (mutated in place) a
    fixed image-space quadrant label via hole_number:
        1 = top-left, 2 = top-right, 3 = bottom-left, 4 = bottom-right.

    Decided design (simpler than an angle-around-centroid sort): the camera
    here is wall-fixed with at most pitch variation -- it never rolls or
    views from a mirrored/opposite angle -- so a plain 2-axis image-space
    split (above/below a horizontal line, left/right of a vertical line) is
    robust, unlike on a moving wrist camera where it wouldn't be.

    Reference point for the horizontal/vertical split: the cup_holder's own
    detected bbox center, if "cup_holder" was present in this frame's
    /detect response (most robust -- one physical object, detected
    independently of how many holes happened to be visible). Falls back to
    the centroid of this frame's own hole detections (mean cx, mean cy) if
    no cup_holder was detected that frame -- still a reasonable reference
    since holes are arranged around the cup_holder.
    """
    holes = [d for d in detections if d.class_name == "hole"]
    if not holes:
        return

    if "cup_holder" in result:
        bbox = result["cup_holder"][0].get("bbox") if result["cup_holder"] else None
    else:
        bbox = None

    if bbox is not None:
        ref_x = (bbox[0] + bbox[2]) / 2.0
        ref_y = (bbox[1] + bbox[3]) / 2.0
    else:
        ref_x = sum(d.cx for d in holes) / len(holes)
        ref_y = sum(d.cy for d in holes) / len(holes)

    for d in holes:
        top = d.cy < ref_y
        left = d.cx < ref_x
        if top and left:
            d.hole_number = 1
        elif top and not left:
            d.hole_number = 2
        elif not top and left:
            d.hole_number = 3
        else:
            d.hole_number = 4


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

        # Detection-resolution downscaling (2026-07-28) -- confirmed live:
        # this project's sim AND real environments are both CPU-only (no
        # GPU, `nvidia-smi` unavailable on either), and YOLO inference is
        # the genuine compute bottleneck (0.4-0.6s/request even with the
        # marker cascade throttled) -- not a bug in any ROS-side code, a
        # hardware/environment throughput ceiling. Extensive live parameter
        # sweeps (confidence_threshold, marker_check_every_n_frames,
        # jpeg_quality, rolling_window_size, stable_drift_threshold_m) all
        # confirmed NOT to move the needle, since none of them reduce the
        # actual per-request YOLO forward-pass cost. A smaller input image
        # genuinely does: fewer pixels for the model to convolve over.
        # 0 = disabled (send the frame at its native resolution, this
        # node's original behavior) -- opt-in, not a silent default change.
        # When set, ANY value > 0 also requires the corresponding rescale-
        # back-to-native-resolution logic in _process_image/
        # publish_detections_2d/publish_marker_pose below -- see
        # scale_intrinsics_and_size()'s own doc comment for why this is
        # handled entirely on THIS side, not inference_server.py's (that
        # server already correctly trusts whatever camera_matrix/image
        # size a caller sends, per aruco_pose.py's own scale_camera_matrix
        # design -- confirmed by reading it directly, not assumed).
        self.detect_max_width_px = int(
            self.get_parameter("detect_max_width_px").value
        )

        # Marker-cascade throttling (2026-07-27) -- live-lab testing found
        # inference_server.py's /detect spending 0.17-0.36s/request on the
        # ArUco marker cascade alone, even when no marker was present,
        # starving cup_holder/hole detection of a smoother/faster stream.
        # Confirmed via a dedicated code-reading pass that
        # calibration_broadcaster_node's per-waypoint sampling (a single
        # blocking wait per waypoint, 5-8s timeout) tolerates this
        # throttle fine, but calibration_orchestrator_node's image-based
        # auto-centering (an iterative rapid move+detect loop) does NOT --
        # see marker_check_full_rate_when_active below for how that's kept
        # safe.
        #
        # 1 = check every frame (this endpoint's original, unthrottled
        # behavior) -- so this param defaults conservatively even before
        # considering the "active" override below.
        self.marker_check_every_n_frames = int(
            self.get_parameter("marker_check_every_n_frames").value
        )
        # When true AND this node's own "active" param is true (this
        # detector is the one currently driving marker_pose for
        # calibration/centering), the cascade always runs at full rate
        # regardless of marker_check_every_n_frames above -- calibration
        # and auto-centering must never be starved by this throttle.
        # Default true: the throttle should only ever relax detection
        # while this detector ISN'T the one calibration/centering depends
        # on; it should never silently apply during an active run.
        self.marker_check_full_rate_when_active = bool(
            self.get_parameter("marker_check_full_rate_when_active").value
        )
        # Incremented once per image_callback invocation -- see
        # image_callback's skip_marker computation below.
        self._frame_count = 0

        # Drop-stale-frames guard (2026-07-27) -- confirmed live: the whole
        # pipeline (overlay_image, detections_2d, stable_positions) was
        # only updating at ~2.7Hz with uneven 0.02-0.67s gaps, all in
        # lockstep, causing everything on the overlay to appear to
        # blink/vanish together. Root cause: image_sub's
        # MutuallyExclusiveCallbackGroup QUEUES every incoming image
        # message while a previous image_callback invocation is still
        # blocked on requests.post() to inference_server.py (bounded by
        # request_timeout_sec, up to 8s on real) -- so by the time a
        # queued frame's turn comes, it's already stale (a much newer
        # camera frame has since arrived), but it still gets processed and
        # published anyway, burning an entire inference cycle on outdated
        # data. Since the true bottleneck is the YOLO model's actual
        # compute time (confirmed via inference_server.py's own timing
        # log: ~0.4-0.6s/request), no amount of ROS-side concurrency makes
        # inference itself faster -- queuing stale frames only adds
        # latency, it never lets the pipeline "catch up". Fix: if a
        # request is already in flight when a new frame arrives, skip that
        # frame immediately instead of waiting to process it later -- see
        # image_callback's guard at its top.
        self._request_in_flight = False

        # Stabilized-overlay feed (2026-07-27) -- depth_perception_node
        # publishes a continuous, gap-free "held position" per instance
        # (see visual_calibration_msgs/StablePositionArray.msg) built from
        # THIS node's own detections_2d stream. Subscribing back to it here
        # is a deliberate exception to this node's otherwise one-directional
        # data flow (image in, poses/detections out): the user explicitly
        # wants ONE overlay image showing both this frame's raw YOLO boxes
        # AND depth_perception's stabilized dots, rather than two separate
        # images to compare. Kept as loosely coupled as that requirement
        # allows: this node only ever reads the published message contract
        # (px/py/drifted/etc.), never depth_perception's internal state, and
        # degrades gracefully (silently draws nothing extra) if
        # depth_perception_node isn't running at all -- see
        # latest_stable_positions_callback/publish_overlay_image_msg.
        self.stable_positions_topic = self.get_parameter(
            "stable_positions_topic"
        ).value
        self._latest_stable_positions = None
        self.stable_positions_sub = self.create_subscription(
            StablePositionArray, self.stable_positions_topic,
            self.stable_positions_callback, 10,
        )

        # "extras" overlay toggle (2026-07-28) -- the magenta/cyan
        # stable_positions markers are OFF by default now (per explicit
        # request to reduce visual complexity: "let's have only the green
        # centroid marker") and only drawn when this is explicitly true --
        # live-toggleable, same get_parameter-every-frame pattern as
        # "active"/"show_centering_crosshair" elsewhere in this project, no
        # restart needed. Eventually flippable from the web app's own
        # "Extras" switch (see CalibrationPanel.tsx) via the standard ROS
        # set_parameters service -- no new topic needed, since this only
        # gates DRAWING of data already flowing on stable_positions_topic.
        self.show_extras_markers = bool(
            self.get_parameter("show_extras_markers").value
        )

        # Calibration-aware suppression (2026-07-28) -- confirmed live: the
        # magenta/cyan stable markers (and, per user request, this SHOULD
        # extend to the raw green marker too during calibration/centering)
        # kept showing a HELD position while the arm physically blocked the
        # camera's view mid-calibration, which read as "still detecting"
        # when nothing fresh was actually being seen. Mirrors
        # depth_perception_node's own calibration_paused_ pattern exactly
        # (same topic, same phase check) -- kept independent rather than
        # shared, since this node has its own separate reason to care
        # (suppressing DRAWING here, not suppressing PROCESSING there).
        self.auto_calibrate_status_topic = self.get_parameter(
            "auto_calibrate_status_topic"
        ).value
        self._calibration_running = False
        self.auto_calibrate_status_sub = self.create_subscription(
            AutoCalibrateStatus, self.auto_calibrate_status_topic,
            self.auto_calibrate_status_callback, 10,
        )

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

        # Separate callback groups (2026-07-24 fixed one live bug via
        # MultiThreadedExecutor + a shared group for both subscriptions;
        # 2026-07-27 fixed a SECOND live bug this introduced) -- see
        # main()'s doc comment for the full MultiThreadedExecutor story
        # (image_callback blocks on a synchronous HTTP request for up to
        # request_timeout_sec, which under single-threaded spin also
        # blocked set_parameters).
        #
        # image_sub and camera_info_sub were originally put in ONE shared
        # MutuallyExclusiveCallbackGroup on the theory that they "don't
        # need to run concurrently with each other" -- confirmed live
        # WRONG: a MutuallyExclusiveCallbackGroup queues every callback
        # sharing it, including different callbacks on different topics.
        # camera_info publishes far more often than image_callback (bounded
        # by request_timeout_sec, up to 3s per frame) can keep up with, so
        # every camera_info message queued behind whichever image_callback
        # was in flight -- and since new ones keep arriving faster than
        # image_callback drains the queue, camera_info_callback could be
        # starved indefinitely. Confirmed live: "No camera_info received
        # yet" repeating forever despite `ros2 topic echo` proving the
        # topic itself was publishing fine the whole time.
        #
        # camera_info_callback is cheap (three numpy assignments, no I/O)
        # and needs to run promptly/often -- it now gets its OWN group, so
        # it's never queued behind an in-flight image_callback. image_sub
        # keeps its own group too (still separate from the node's default
        # group, preserving the 2026-07-24 fix for set_parameters).
        self._image_callback_group = MutuallyExclusiveCallbackGroup()
        self._camera_info_callback_group = MutuallyExclusiveCallbackGroup()
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback,
            qos_profile_sensor_data,
            callback_group=self._image_callback_group,
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback,
            qos_profile_sensor_data,
            callback_group=self._camera_info_callback_group,
        )

        self.get_logger().info(
            "yolo_marker_bridge_node ready (image_topic: '%s', "
            "camera_info_topic: '%s', pose_topic: '%s', "
            "detections_2d_topic: '%s', inference_server_url: '%s', "
            "marker_check_every_n_frames: %d, "
            "marker_check_full_rate_when_active: %s, "
            "stable_positions_topic: '%s', show_extras_markers: %s, "
            "auto_calibrate_status_topic: '%s')" % (
                self.image_topic, self.camera_info_topic, self.pose_topic,
                self.detections_2d_topic, self.inference_server_url,
                self.marker_check_every_n_frames,
                self.marker_check_full_rate_when_active,
                self.stable_positions_topic,
                self.show_extras_markers,
                self.auto_calibrate_status_topic,
            )
        )

    def camera_info_callback(self, msg):
        # Always refresh (unlike aruco_detector_node.cpp, which latches on
        # first receipt) -- the inference server takes intrinsics fresh on
        # every request rather than caching them server-side, so mirror
        # that "always current" contract on this side too.
        self.camera_matrix = np.array(msg.k, dtype=float).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=float)

    def stable_positions_callback(self, msg):
        # Just caches the latest message for publish_overlay_image_msg to
        # read -- no processing here, matching camera_info_callback's own
        # "always refresh, never derive anything in the callback itself"
        # pattern. If depth_perception_node is never running,
        # self._latest_stable_positions simply stays None forever and the
        # overlay silently draws nothing extra for it -- see
        # publish_overlay_image_msg's own guard.
        self._latest_stable_positions = msg

    def auto_calibrate_status_callback(self, msg):
        # Mirrors depth_perception_node's own autoCalibrateStatusCallback
        # pattern exactly (same topic, same PHASE_RUNNING check) -- see
        # self._calibration_running's own doc comment in __init__ for why
        # this node tracks it independently rather than sharing state with
        # depth_perception_node.
        was_running = self._calibration_running
        self._calibration_running = (msg.phase == AutoCalibrateStatus.PHASE_RUNNING)
        if self._calibration_running and not was_running:
            self.get_logger().info(
                "~/auto_calibrate started (stage: '%s') -- suppressing overlay "
                "markers until it finishes." % msg.stage
            )
        elif not self._calibration_running and was_running:
            self.get_logger().info(
                "~/auto_calibrate finished (success: %s) -- resuming overlay markers."
                % msg.success
            )

    def image_callback(self, msg):
        # Drop-stale-frames guard -- see self._request_in_flight's own
        # doc comment in __init__ for the full rationale. Checked BEFORE
        # any other work (camera_matrix check, cv_bridge conversion, etc)
        # so a frame that arrives mid-request costs nothing beyond this
        # one boolean check, rather than being queued and processed stale
        # later by image_sub's MutuallyExclusiveCallbackGroup.
        if self._request_in_flight:
            return
        self._request_in_flight = True
        try:
            self._process_image(msg)
        finally:
            self._request_in_flight = False

    def _process_image(self, msg):
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

        # Detection-resolution downscaling (2026-07-28) -- see
        # detect_max_width_px's own __init__ comment for the full
        # rationale. detect_image/detect_camera_matrix are what's actually
        # SENT to inference_server.py; cv_image itself is left untouched
        # (still needed at full resolution for the overlay draw below).
        # rescale_factor is applied to every 2D pixel field in the
        # response before anything downstream (marker_pose, detections_2d,
        # overlay) ever sees it -- see the rescale block after the request
        # completes.
        detect_image = cv_image
        detect_camera_matrix = self.camera_matrix
        rescale_factor = 1.0
        if self.detect_max_width_px > 0 and cv_image.shape[1] > self.detect_max_width_px:
            rescale_factor = cv_image.shape[1] / self.detect_max_width_px
            new_width = self.detect_max_width_px
            new_height = int(round(cv_image.shape[0] / rescale_factor))
            detect_image = cv2.resize(
                cv_image, (new_width, new_height), interpolation=cv2.INTER_AREA
            )
            # fx/fy/cx/cy scale down by the same factor as the image --
            # see aruco_pose.py's scale_camera_matrix, which does the exact
            # same proportional scaling; not reused directly here since
            # that function lives in the isolated YOLO venv (never
            # imported into this rclpy process, per this project's locked
            # ABI-isolation architecture) -- this is the small, standard
            # pinhole-intrinsics scaling formula, safe to duplicate rather
            # than cross an intentional process boundary for it.
            detect_camera_matrix = self.camera_matrix.copy()
            detect_camera_matrix[0, 0] /= rescale_factor  # fx
            detect_camera_matrix[1, 1] /= rescale_factor  # fy
            detect_camera_matrix[0, 2] /= rescale_factor  # cx
            detect_camera_matrix[1, 2] /= rescale_factor  # cy

        ok, jpeg_bytes = cv2.imencode(
            ".jpg", detect_image,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not ok:
            self.get_logger().error(
                "cv2.imencode failed to JPEG-encode the frame -- skipping.",
                throttle_duration_sec=5.0,
            )
            return

        image_b64 = base64.b64encode(jpeg_bytes.tobytes()).decode("ascii")

        # Marker-cascade throttling decision (2026-07-27) -- see
        # marker_check_every_n_frames/marker_check_full_rate_when_active's
        # own comments in __init__ for the full rationale. "active" is
        # re-read live here (not cached), matching the same fresh-read
        # pattern already used for the marker_pose-publish gate below --
        # so a calibration_orchestrator_node set_parameters call flipping
        # "active" takes effect on the very next frame for this override
        # too, not just for the pose-publish gate.
        self._frame_count += 1
        full_rate_forced = (
            self.marker_check_full_rate_when_active
            and bool(self.get_parameter("active").value)
        )
        skip_marker = (
            not full_rate_forced
            and (self._frame_count % self.marker_check_every_n_frames != 0)
        )

        request_body = {
            "image_jpeg_base64": image_b64,
            "camera_matrix": detect_camera_matrix.tolist(),
            "dist_coeffs": self.dist_coeffs.tolist(),
            "conf": self.confidence_threshold,
            "skip_marker": skip_marker,
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

        # Rescale every 2D pixel field back to cv_image's TRUE native
        # resolution -- a no-op (rescale_factor == 1.0) when
        # detect_max_width_px is disabled/didn't trigger this frame. Done
        # ONCE, here, so every downstream consumer (publish_marker_pose,
        # publish_detections_2d, publish_overlay_image_msg) keeps working
        # against full-resolution pixel coordinates with zero changes of
        # its own -- none of them need to know downscaling happened.
        # aruco_marker's rvec/tvec (a 3D metric position/orientation, not
        # pixels) is DELIBERATELY left untouched: solvePnP already used
        # detect_camera_matrix (scaled to match the downscaled image), so
        # that result is already correct in real-world units -- only
        # "corners" (raw pixel coordinates) needs this correction.
        if rescale_factor != 1.0:
            if "aruco_marker" in result:
                result["aruco_marker"]["corners"] = [
                    [x * rescale_factor, y * rescale_factor]
                    for x, y in result["aruco_marker"]["corners"]
                ]
            for class_name in ("cup_holder", "hole"):
                for detection in result.get(class_name, []):
                    detection["cx"] *= rescale_factor
                    detection["cy"] *= rescale_factor
                    detection["bbox"] = [v * rescale_factor for v in detection["bbox"]]

        # classical/hybrid switch: only publish the marker pose when this
        # node is the active detector -- live re-read, never cached, see
        # class doc comment / the "active" parameter's declare_parameter
        # comment above.
        if "aruco_marker" in result and self.get_parameter("active").value:
            self.publish_marker_pose(msg, result["aruco_marker"])

        # Overlay: independent of "active" -- an operator debugging hybrid
        # mode still wants the visual confirmation even if this node isn't
        # currently the one publishing marker_pose (see publish_overlay_image's
        # declare_parameter comment above). Published whenever there's
        # anything to draw -- aruco_marker corners/axes, per-hole quadrant
        # labels, OR depth_perception's stabilized positions (2026-07-27) --
        # that last condition is why a stabilized dot can still be drawn/
        # published even on a frame where THIS frame's result has neither
        # aruco_marker nor hole at all.
        if self.overlay_image_pub is not None and (
            "aruco_marker" in result or "hole" in result
            or self._latest_stable_positions is not None
        ):
            self.publish_overlay_image_msg(msg, cv_image, result)

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
                det.hole_number = 0  # unset/not-applicable by default
                array_msg.detections.append(det)

        # Quadrant-label every "hole" entry just added above (aruco_marker/
        # cup_holder are left at hole_number=0 -- only one of each ever
        # exists in frame, no ambiguity to label). See assign_hole_quadrants'
        # own doc comment for the exact rule.
        assign_hole_quadrants(array_msg.detections, result)

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

    def publish_overlay_image_msg(self, image_msg, cv_image, result):
        """Yellow border + XYZ axes overlay for aruco_marker (matching
        aruco_detector_node.cpp's classical overlay_image exactly -- same
        drawDetectedMarkers/drawFrameAxes calls, same
        overlay_border_color_bgr default, same bgr8 encoding -- so a viewer/
        consumer of /aruco_perception/overlay_image sees identical visuals
        regardless of which detector produced it), PLUS each "hole"
        detection's quadrant label (see assign_hole_quadrants) drawn as text
        near its centroid -- the label needs to land on this same overlay
        image, which is why it's computed here in the bridge/producer node
        rather than downstream in a consumer. Draws on a COPY of cv_image
        (never mutates the frame used for the /detect request above).

        Calibration/self-centering suppression (2026-07-28): while
        self._calibration_running is true (an ~/auto_calibrate run is
        actively in progress, including its auto-centering stage -- see
        auto_calibrate_status_callback), NO markers of any kind are drawn
        -- confirmed live that a HELD/stable marker looked indistinguishable
        from a genuinely fresh detection while the arm physically blocked
        the camera's view mid-calibration, misleadingly reading as "still
        detecting". The plain camera frame still publishes underneath (same
        "never stop the stream, just stop drawing on it" convention as the
        marker-not-found case below) so a viewer doesn't see a frozen image,
        just a temporarily plain one.
        """
        overlay = cv_image.copy()

        if self._calibration_running:
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
            return

        if "aruco_marker" in result:
            marker_result = result["aruco_marker"]
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

        if "hole" in result:
            # Recompute the same quadrant assignment used for the published
            # Detection2DArray (cheap, keeps this function independent of
            # publish_detections_2d's own Detection2D list) so the overlay's
            # drawn labels always match what was published this frame.
            holes = [Detection2D() for _ in result["hole"]]
            for det, d in zip(holes, result["hole"]):
                det.class_name = "hole"
                det.cx = float(d.get("cx", 0.0))
                det.cy = float(d.get("cy", 0.0))
            assign_hole_quadrants(holes, result)

            for det in holes:
                label = str(det.hole_number)
                text_pos = (int(det.cx) + 8, int(det.cy) - 8)
                # Black outline + green fill for readability against any
                # background (matches the "clear/readable" requirement --
                # a single solid color alone can wash out against a light
                # cup_holder surface).
                cv2.putText(
                    overlay, label, text_pos, cv2.FONT_HERSHEY_SIMPLEX,
                    0.9, (0, 0, 0), 3, cv2.LINE_AA,
                )
                cv2.putText(
                    overlay, label, text_pos, cv2.FONT_HERSHEY_SIMPLEX,
                    0.9, (0, 255, 0), 2, cv2.LINE_AA,
                )
                cv2.circle(overlay, (int(det.cx), int(det.cy)), 4, (0, 255, 0), -1)

        # depth_perception_node's stabilized positions (2026-07-27) --
        # drawn in a DISTINCT color (cyan/magenta) from the green raw-YOLO
        # markers above, specifically so the two are visually
        # distinguishable on one image: this is the "held last known
        # position" the flicker-fix mechanism produces, not this frame's
        # raw detection. Drawn regardless of whether "hole"/"aruco_marker"
        # were present in THIS frame's result at all -- that's the whole
        # point of subscribing to a continuous stream instead of deriving
        # this from `result` -- so a stabilized dot keeps showing even on
        # a frame where YOLO found nothing. No-op (nothing extra drawn) if
        # depth_perception_node has never published anything yet.
        #
        # Gated behind show_extras_markers (2026-07-28, default OFF) -- per
        # explicit request to reduce visual complexity ("let's have only
        # the green centroid marker") -- live re-read every frame (never
        # cached), same pattern as "active", so a future web "Extras"
        # switch takes effect on the very next frame with no restart.
        self.show_extras_markers = bool(self.get_parameter("show_extras_markers").value)
        if self.show_extras_markers and self._latest_stable_positions is not None:
            for position in self._latest_stable_positions.positions:
                px, py = int(position.px), int(position.py)
                # Magenta = held (unchanged since last real update), cyan =
                # just drifted (this exact update moved the position) --
                # lets an operator visually confirm real movement is being
                # tracked, not just that a dot exists.
                color = (255, 255, 0) if position.drifted else (255, 0, 255)
                cv2.circle(overlay, (px, py), 8, color, 2)
                if position.class_name == "hole":
                    label = "h%d*" % position.hole_number
                else:
                    label = "%s*" % position.class_name
                text_pos = (px + 8, py + 20)
                cv2.putText(
                    overlay, label, text_pos, cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 0, 0), 3, cv2.LINE_AA,
                )
                cv2.putText(
                    overlay, label, text_pos, cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, color, 1, cv2.LINE_AA,
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