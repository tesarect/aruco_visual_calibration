[← Back to index](./README.md)

# aruco_perception_yolo_bridge — class docs

Classes documented here: `YoloMarkerBridgeNode`. Plus the free function
`rotation_matrix_to_quaternion`, covered under its own section since it's
not a class. See [../aruco_perception_yolo_bridge.md](../aruco_perception_yolo_bridge.md)
for the plain-language explanation of why this package exists, the
classical/hybrid detector switch, and the HTTP contract with
`inference_server.py`.

---

## rotation_matrix_to_quaternion

Free function. Standard rotation-matrix → quaternion `(x, y, z, w)`
conversion (Shepperd's method, the same "largest diagonal element" branch
`tf2`'s own `Matrix3x3::getRotation` uses), reimplemented in plain numpy
rather than adding a new `scipy`/`tf_transformations` dependency for one
well-known ~15-line formula.

Parameters: `rotation_matrix`

---

## YoloMarkerBridgeNode

```mermaid
classDiagram
    class YoloMarkerBridgeNode {
        +YoloMarkerBridgeNode()
        -camera_info_callback(msg) void
        -image_callback(msg) void
        -_process_image(msg) void
        -_send_detect_request(cv_image, skip_marker) tuple
        -handle_detect_marker_once(request, response) DetectMarkerOnce.Response
        -_publish_hybrid_pip_overlay(image_msg, cv_image, marker_result) void
        -assign_hole_quadrants(hole_dicts) list
        -publish_detections_2d(image_msg, result) void
        -publish_marker_pose(image_msg, marker_result) void
        -publish_overlay_image_msg(image_msg, cv_image, result) void
        -stable_positions_callback(msg) void
        -auto_calibrate_status_callback(msg) void
        -camera_matrix numpy.ndarray
        -dist_coeffs numpy.ndarray
        -pose_pub Publisher
        -detections_2d_pub Publisher
        -overlay_image_pub Publisher
        -detect_marker_once_srv Service
    }
```

Subscribes to the camera's image/`camera_info` topics, calls the YOLO
inference server over HTTP, and republishes results onto the ROS graph — a
drop-in alternative to `aruco_perception`'s classical `ArucoDetectorNode`
for the marker-pose case, plus new `cup_holder`/`hole` 2D detections the
classical detector has no equivalent of. See
[../aruco_perception_yolo_bridge.md](../aruco_perception_yolo_bridge.md)
for the full per-frame behavior and why this process never imports
`ultralytics`.

### YoloMarkerBridgeNode

Constructs the node with `automatically_declare_parameters_from_overrides=True`
(no explicit `declare_parameter` calls — every parameter comes from the
`--params-file` yaml; adding an explicit `declare_parameter` for an
already-yaml-declared name throws `ParameterAlreadyDeclaredException`, a
live-crash lesson this file follows the same convention
`ArucoDetectorNode` already used to avoid). Creates the image/`camera_info`
subscriptions on one dedicated `MutuallyExclusiveCallbackGroup`, separate
from the node's default group the `set_parameters` service runs in — see
`main`'s doc comment for why.

### camera_info_callback

Always refreshes `camera_matrix`/`dist_coeffs` from the latest message
(unlike the classical detector, which latches on first receipt) — mirrors
`inference_server.py`'s own "always current, never cached" intrinsics
contract.

Parameters: `msg`

### image_callback

Drop-stale-frames guard first: if a previous invocation's `/detect` call
is still in flight (`_request_in_flight`), returns immediately rather
than letting `image_sub`'s callback group queue this frame for later,
stale, processing — see
[../aruco_perception_yolo_bridge.md](../aruco_perception_yolo_bridge.md)'s
"CPU-budget mitigations" section. Otherwise delegates to `_process_image`.

Parameters: `msg`

### _process_image / _send_detect_request

`_process_image` is `image_callback`'s shared core: converts the image via
`cv_bridge`, caches the latest decoded frame for
`handle_detect_marker_once`'s "wait for a fresh frame" guard, computes the
per-frame `skip_marker` throttling decision
(`marker_check_every_n_frames`/`marker_check_full_rate_when_active`), calls
`_send_detect_request`, then — if `aruco_marker` is present in the
response — publishes the marker pose (only if `active` is true) and the
debug overlay (only if there's anything to draw), and always publishes
`cup_holder`/`hole`/`aruco_marker` detections onto `detections_2d_topic`
regardless of `active`. A failed `cv_bridge` conversion or a failed
`_send_detect_request` call each log and skip the frame without crashing
the node.

`_send_detect_request` builds and POSTs the `/detect` request body —
factored out so both the continuous per-frame path and
`handle_detect_marker_once`'s on-demand path share one HTTP call
implementation instead of two copies. Applies `detect_max_width_px`
downscaling to the outgoing frame/intrinsics if configured, and rescales
every 2D pixel field in the response back to the source frame's native
resolution before returning it — see
[../aruco_perception_yolo_bridge.md](../aruco_perception_yolo_bridge.md)'s
"CPU-budget mitigations" section for the full rationale.

Parameters: `cv_image`, `skip_marker`

### handle_detect_marker_once

`~/detect_marker_once` service handler — see
[../aruco_perception_yolo_bridge.md](../aruco_perception_yolo_bridge.md)'s
own section for the full mechanism (waits for a genuinely fresh frame,
forces the cascade to run, does not publish to `marker_pose`/`detections_2d`,
does publish a picture-in-picture overlay update on success).

Parameters: `request`, `response`

### _publish_hybrid_pip_overlay

Draws a small picture-in-picture inset of the cascade's winning crop onto
`overlay_image`, called from `handle_detect_marker_once`'s success path —
lets a human watching the live overlay feed see what the per-waypoint
hybrid cascade detected during a run, since `publish_overlay_image_msg`'s
own continuous-mode drawing is suppressed for the run's duration.

Parameters: `image_msg`, `cv_image`, `marker_result`

### assign_hole_quadrants

Labels each hole 1–4 by quadrant relative to the frame's cupholder/hole
group center — see [../aruco_perception_yolo_bridge.md](../aruco_perception_yolo_bridge.md)
and `cup_holder_detector_node`'s own `assignHoleQuadrants` (a sim-side
C++ port of this same function, with one deliberate divergence — see that
method's own doc comment in
[aruco_perception.md](./aruco_perception.md#assignholequadrants)).

Parameters: `hole_dicts`

### stable_positions_callback / auto_calibrate_status_callback

Cache the latest `depth_perception_node` stable-position stream and
`calibration_orchestrator_node`'s auto-calibrate status respectively —
feed the overlay's stabilized-position markers and the "suppress overlay
drawing during a hybrid run" behavior `_publish_hybrid_pip_overlay`
depends on.

Parameters: `msg`

### publish_detections_2d

Builds and publishes one `visual_calibration_msgs/Detection2DArray`
containing whichever of `aruco_marker`/`cup_holder`/`hole` were present in
the inference response — always publishes, every frame, even with an empty
`detections[]`, so consumers see a continuous stream rather than needing to
distinguish "nothing detected" from "node down."

Parameters: `image_msg`, `result`

### publish_marker_pose

Converts the response's Rodrigues rotation vector to a quaternion (via
`rotation_matrix_to_quaternion`) and publishes `PoseStamped` on
`pose_topic`, reusing the source image message's own header (stamp +
`frame_id`) — same convention as the classical detector, not
`self.get_clock().now()`.

Parameters: `image_msg`, `marker_result`

### publish_overlay_image_msg

Draws the marker's border and XYZ axes onto a **copy** of the frame (never
mutates the frame already sent to the inference server) and publishes it —
same `drawDetectedMarkers`/`drawFrameAxes` calls, overlay color default,
and encoding as the classical detector's own overlay, so a viewer sees
identical visuals regardless of which detector produced them.

Parameters: `image_msg`, `cv_image`, `marker_result`

### main

Runs a `MultiThreadedExecutor`, not plain `rclpy.spin()` — `image_callback`
blocks on a synchronous HTTP request for up to `request_timeout_sec`, which
under single-threaded spin would also block the standard `set_parameters`
service callback `calibration_orchestrator_node` uses to flip `active`
(confirmed live: that call timed out even though the node was genuinely
healthy). A multi-threaded executor lets the parameter-service callback run
concurrently on a different thread instead of queueing behind an in-flight
HTTP request.
