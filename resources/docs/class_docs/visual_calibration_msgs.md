[← Back to index](../README.md)

# visual_calibration_msgs — interface docs

Interfaces-only package — no C++ classes, just `.srv`/`.action`/`.msg`
definitions shared between `aruco_perception`, `aruco_perception_yolo_bridge`,
`orchestrator`, `depth_perception`, and `visual_calibration_moveit`.
Documented here as field tables instead of class diagrams.

---

## Calibrate.action

Goal-driven `~/calibrate` action, served by `CalibrationBroadcasterNode`
(see [aruco_perception.md](./aruco_perception.md)). Starts an N-sample
calibration run and reports progress as each sample is collected.

**Goal** — empty. N (the sample count) is a server-side parameter
(`CalibrationBroadcasterConfig::num_samples`), not a per-call argument —
matching the project's convention of tuning via YAML, not call sites.

**Result**

| Field | Type | Meaning |
|---|---|---|
| `success` | `bool` | Whether the run completed all samples without aborting. |
| `message` | `string` | Human-readable status/error detail. |
| `max_spread_deg` | `float64` | Largest single-sample deviation from the averaged orientation. |
| `mean_spread_deg` | `float64` | Average sample deviation from the averaged orientation. |
| `is_high_confidence` | `bool` | Soft, informational signal — `true` if the post-outlier-rejection spread is within `position_spread_tolerance_cm`/`orientation_spread_tolerance_deg`. Never affects `success`; lets a caller (web UI) show a warning without blocking the workflow. |

The computed `known_chain_frame → camera` transform itself isn't included
here — it's already broadcast on `/tf` by the time the action completes;
read it from there.

**Feedback**

| Field | Type | Meaning |
|---|---|---|
| `samples_collected` | `uint32` | How many samples have been accepted so far. |
| `samples_total` | `uint32` | Total samples this run will collect. |
| `current_status` | `string` | Short human-readable text for the current event, e.g. a discard-and-continue/timeout message from `min_samples_to_finish`'s soft-fail path. Empty for a normal "sample recorded" event. |
| `latest_sample_pose` | `geometry_msgs/Pose` | The just-recorded sample's `known_chain_frame → camera` candidate pose — lets a caller visualize the estimate converging sample by sample, not just the final average. |

---

## TracePath.srv

Blocking `~/trace_path` service, served by `TrajectoryPlanner` (see
[visual_calibration_moveit.md](./visual_calibration_moveit.md)). Moves the
end-effector through a list of waypoints in order.

**Request**

| Field | Type | Meaning |
|---|---|---|
| `waypoints` | `geometry_msgs/Pose[]` | Poses to visit in order, in the planning frame. |
| `planning_mode` | `uint8` | `PLANNING_MODE_JOINT_SPACE` (0) or `PLANNING_MODE_CARTESIAN` (1, default). |
| `pose_name` | `string` | Optional: name of the destination pose (e.g. `"home"`, `"cal_ready"`), for `~/current_pose_name` reporting. Only applies to single-waypoint calls; ignored for multi-waypoint calls; leave empty for arbitrary/unnamed waypoints (e.g. calibration polygon corners). |
| `is_sequenced_goal` | `bool` | Optional, default `false`. Marks a single-waypoint call as one of the arm's regular working stops (as opposed to a special named stop like `"home"`/`"cal_ready"`/`"standby"`) — opts it into `TrajectoryPlanner`'s automatic stay/lift/standby sequence after the move (see `ArmState` in [visual_calibration_moveit.md](./visual_calibration_moveit.md)). Every existing caller is unaffected unless it explicitly opts in. |

`PLANNING_MODE_CARTESIAN` moves in a straight line between waypoints —
predictable geometry, useful for calibration, but can fail partway
(collision/IK/joint-limit). `PLANNING_MODE_JOINT_SPACE` is the older,
more-robust-but-unpredictable-path fallback.

**Response**

| Field | Type | Meaning |
|---|---|---|
| `success` | `bool` | Whether every waypoint was reached. |
| `message` | `string` | Human-readable status/error detail. |

Returns once all waypoints are visited, or on the first planning/execution
failure — no partial-success reporting per waypoint.

---

## GetPolygonWaypoints.srv

Read-only `~/get_polygon_waypoints` service, served by `TrajectoryPlanner`.
Computes and returns the polygon waypoints around the arm's own current
pose **without moving the arm**, so a caller (e.g.
`CalibrationBroadcasterNode`) can drive them one at a time itself via
`TracePath.srv`, sampling between moves — without duplicating
`TrajectoryPlanner`'s standoff/polygon geometry or its config
(`camera_frame`, `standoff_m`, `facing_rpy_rad`, `polygon_num_corners`,
`polygon_radius_m` all stay owned by `TrajectoryPlanner`).

**Request** — empty.

**Response**

| Field | Type | Meaning |
|---|---|---|
| `success` | `bool` | Whether the waypoints were computed (e.g. TF lookup succeeded). |
| `message` | `string` | Human-readable status/error detail. |
| `waypoints` | `geometry_msgs/Pose[]` | The computed polygon waypoints, in angular order. |
| `center_pose` | `geometry_msgs/Pose` | The center the waypoints were generated around — the arm's own current pose at call time, not a TF-derived standoff pose (redesigned so a caller doing its own additional offset sampling, e.g. `CalibrationBroadcasterNode`'s random phase, can reuse the same center without a second "get current pose" round-trip). |

---

## GetStandoffPose.srv

Read-only `~/get_standoff_pose` service, served by `TrajectoryPlanner`.
Computes and returns the TF-derived standoff pose **without moving the
arm**, so a caller can check whether a deterministic standoff pose is
available before deciding whether to move there or fall back to a preset.

**Request** — empty.

**Response**

| Field | Type | Meaning |
|---|---|---|
| `success` | `bool` | `true` if either a live TF lookup or a `"standoff"` preset was available — `false` only if neither was. |
| `used_fallback` | `bool` | `true` if `camera_frame` TF wasn't available and `standoff_pose` was instead read from the `"standoff"` entry in `preset_poses_{sim,real}.yaml`. `success` can still be `true` in that case. |
| `message` | `string` | Human-readable status/error detail. |
| `standoff_pose` | `geometry_msgs/Pose` | The computed (or fallback) standoff pose. |

---

## GetPresetPose.srv

Read-only `~/get_preset_pose` service, served by `TrajectoryPlanner`.
Returns a named preset pose from `preset_poses_{sim,real}.yaml` **without
moving the arm**.

**Request**

| Field | Type | Meaning |
|---|---|---|
| `name` | `string` | Preset name (e.g. `"standoff"`, `"home"`, `"cal_ready"` — any entry under that yaml's `preset_names`). |

**Response**

| Field | Type | Meaning |
|---|---|---|
| `success` | `bool` | Whether a preset with that name was loaded. |
| `message` | `string` | Human-readable status/error detail. |
| `pose` | `geometry_msgs/Pose` | The preset's Cartesian pose, for `end_effector_frame` in the planning frame. |

---

## MoveToPreset.srv

`~/move_to_preset` service, served by `TrajectoryPlanner`. **Moves the
arm** to a named preset from `preset_poses_{sim,real}.yaml`.

**Request**

| Field | Type | Meaning |
|---|---|---|
| `name` | `string` | Preset name (e.g. `"home"`, `"standby"`, `"cal_ready"`). |

**Response**

| Field | Type | Meaning |
|---|---|---|
| `success` | `bool` | Whether the move succeeded. |
| `message` | `string` | Human-readable status/error detail. |

Prefers a joint-value preset (pins the exact IK branch — see
`TrajectoryPlanner::planAndExecuteToPreset`) if one is loaded for `name`;
otherwise falls back to the Cartesian pose preset if one is loaded instead.
Fails if neither exists for `name` — never consults TF at all (unlike
`GetStandoffPose.srv`).

---

## SetDetectorMode.srv

`~/set_detector_mode` service, served by `CalibrationOrchestratorNode`.
Switches which of the two marker detectors — classical
(`aruco_perception`'s `ArucoDetectorNode`) or hybrid
(`aruco_perception_yolo_bridge`'s `YoloMarkerBridgeNode`) — actively
publishes `geometry_msgs/PoseStamped` on `/aruco_perception/marker_pose`.

**Request**

| Field | Type | Meaning |
|---|---|---|
| `mode` | `string` | `"classical"` or `"hybrid"` — any other value is rejected. Not a `bool`, so a future third mode isn't a breaking change to this service's shape. |

**Response**

| Field | Type | Meaning |
|---|---|---|
| `success` | `bool` | Whether the switch was applied. |
| `message` | `string` | Human-readable status/error detail. |

Implemented by flipping exactly one of the two detector nodes' `active`
boolean parameter true (and the other false) via the standard ROS
`set_parameters` service — no lifecycle nodes, no process start/stop; both
nodes stay running/subscribed the whole time. `yolo_marker_bridge_node`
keeps publishing `cup_holder`/`hole` detections continuously regardless of
this switch — see [aruco_perception_yolo_bridge.md](../aruco_perception_yolo_bridge.md).

---

## DetectMarkerOnce.srv

On-demand, single-shot `~/detect_marker_once` service, served by
`aruco_perception_yolo_bridge`'s `YoloMarkerBridgeNode`. Runs one YOLO
crop + image-enhancement cascade + classical ArUco + `solvePnP` detection
against the next fresh camera frame, for
`calibration_broadcaster_node`'s `hybrid_per_waypoint_enabled` mode — see
[aruco_perception_yolo_bridge.md](../aruco_perception_yolo_bridge.md)'s
own section for the full mechanism.

**Request** — empty.

**Response**

| Field | Type | Meaning |
|---|---|---|
| `success` | `bool` | Whether a marker was found. |
| `message` | `string` | Human-readable status/error detail. |
| `marker_pose` | `geometry_msgs/PoseStamped` | The detected camera → marker pose, same shape `publish_marker_pose` builds for continuous mode. |
| `cascade_variant_used` | `string` | Which enhancement cascade variant succeeded (e.g. `"gamma_0.7"`). Empty if `success` is `false`. |
| `cascade_image_b64` | `string` | The winning crop image, JPEG/base64 — an inspection artifact `calibration_broadcaster_node` accumulates into one labeled debug grid per run. Empty if `success` is `false`. |
| `failure_reason` | `string` | Set only when `success` is `false`: `"no_yolo_bbox"` (YOLO found no candidate box at all) or `"no_classical_match"` (YOLO found a box, but classical detection failed on every enhancement variant tried within it). |
| `detect_time_s` | `float64` | How long the YOLO+cascade call itself took — excludes this service call's round-trip and any SIGCONT/SIGSTOP overhead. Only meaningful if `success` is `true`. |

---

## SignalInferenceServer.srv

`~/signal_inference_server` service, served by
`CalibrationOrchestratorNode`. Sends SIGSTOP or SIGCONT to the running
`inference_server.py` process (a thin wrapper around
`signalInferenceServer()`) — exposed as a service so
`calibration_broadcaster_node` (a different package) can reuse the same
`/proc`-scan-and-signal implementation at a per-waypoint grain for
`hybrid_per_waypoint_enabled` mode, instead of a second, divergent
implementation. See [orchestrator.md](../orchestrator.md)'s "Pausing YOLO
inference during a run" section.

**Request**

| Field | Type | Meaning |
|---|---|---|
| `stop` | `bool` | `true` = SIGSTOP (pause), `false` = SIGCONT (resume). |

**Response**

| Field | Type | Meaning |
|---|---|---|
| `success` | `bool` | Whether the signal was sent. |
| `message` | `string` | Human-readable status/error detail. |

---

## MoveToInstance.srv

`~/move_to_instance` service. Moves the arm to a **live TF lookup** of a
named cupholder/hole instance (`"cup_holder"`, `"hole_1".."hole_4"` — the
exact `child_frame_id`s `depth_perception_node::broadcastInstanceTfs()`
publishes on `/tf`), not a static preset — these positions are only known
once calibration and depth-perception have actually run, and would go
stale in a YAML file if the camera/table setup changes.

**Request**

| Field | Type | Meaning |
|---|---|---|
| `instance_name` | `string` | `"cup_holder"` or `"hole_1".."hole_4"`. |

**Response**

| Field | Type | Meaning |
|---|---|---|
| `success` | `bool` | Whether the move succeeded. |
| `message` | `string` | Human-readable status/error detail — includes a clear failure if the TF doesn't exist yet (no `~/calibrate` run completed this session, or `depth_perception_node` hasn't detected this instance yet). |

---

## AutoCalibrate.action

Goal-driven `~/auto_calibrate` action, served by
`CalibrationOrchestratorNode` (see [orchestrator.md](../orchestrator.md)).
Chains "move to cal_ready", "optionally auto-center on the marker", and
"call `Calibrate.action`" into one action.

**Goal** — empty. Whether Stage 1 moves to `cal_ready` or instead
calibrates from the arm's current pose is decided internally by the node
(see `pending_manual_adjustment_` in
[class_docs/orchestrator.md](./orchestrator.md)), not a per-call goal
field.

**Result**

| Field | Type | Meaning |
|---|---|---|
| `success` | `bool` | Whether the full sequence completed successfully. |
| `message` | `string` | Human-readable status/error detail. |
| `max_spread_deg` | `float64` | Mirrors `Calibrate.action`'s result field. |
| `mean_spread_deg` | `float64` | Mirrors `Calibrate.action`'s result field. |
| `failed_stage` | `string` | Empty on success; one of `"cal_ready"`, `"auto_center"`, `"calibrate"` on failure — lets a caller show a more specific error than `message` alone. |

**Feedback**

| Field | Type | Meaning |
|---|---|---|
| `stage` | `string` | One line per stage transition, e.g. `"Moving to cal_ready"`, `"Auto-centering on marker"`, `"Calibrating (sample 3/10)"` — the last form mirrors `Calibrate.action`'s own feedback once that stage is reached. |
| `samples_collected` | `uint32` | Meaningful once the calibrate stage is reached. |
| `samples_total` | `uint32` | Meaningful once the calibrate stage is reached. |

---

## AutoCalibrateStatus.msg

Published on `calibration_orchestrator_node`'s `~/auto_calibrate_status`
topic, one message per `AutoCalibrate.action` feedback/result event —
mirrors that action's Feedback/Result fields exactly, but reachable from
clients that can't speak rosbridge's native ROS2 action protocol (this
project's `rosbridge_suite` 1.3.1 has no action support at all). Start the
sequence via `~/start_auto_calibrate` (`std_srvs/Trigger`) instead of
sending an action goal directly, then subscribe to this topic for
progress/result — see [orchestrator.md](../orchestrator.md).

| Field | Type | Meaning |
|---|---|---|
| `phase` | `uint8` | `PHASE_RUNNING` (0), `PHASE_SUCCEEDED` (1), or `PHASE_FAILED` (2). |
| `stage` | `string` | Feedback field, meaningful while `phase == PHASE_RUNNING`. |
| `samples_collected` | `uint32` | Feedback field. |
| `samples_total` | `uint32` | Feedback field. |
| `success` | `bool` | Result field, meaningful once `phase != PHASE_RUNNING`. |
| `message` | `string` | Result field. |
| `max_spread_deg` | `float64` | Result field. |
| `mean_spread_deg` | `float64` | Result field. |
| `failed_stage` | `string` | Result field. |

Plain reliable QoS, not `transient_local` — a caller must
`~/start_auto_calibrate` first, then subscribe, to see that run's progress;
there is no meaningful "last status" to replay across separate runs.

---

## Detection2D.msg / Detection2DArray.msg

A single YOLO-detected instance (`Detection2D`) and its array wrapper
(`Detection2DArray`), published by `aruco_perception_yolo_bridge`'s
`YoloMarkerBridgeNode` (and, for the `"aruco_marker"` class only, also by
`aruco_perception`'s classical `ArucoDetectorNode`) on
`/aruco_perception/detections_2d`. **Not** `vision_msgs/Detection2DArray` —
`vision_msgs` is not a dependency anywhere in this workspace, and its
`Detection2D` carries bounding-box-and-hypothesis/pose/covariance fields
this project doesn't need; these types are named similarly for readability
but are not interchangeable with `vision_msgs`'.

**Detection2DArray**

| Field | Type | Meaning |
|---|---|---|
| `header` | `std_msgs/Header` | Same convention as `marker_pose`: reuses the source image message's own header, not `now()`. |
| `detections` | `Detection2D[]` | One entry per detected instance this frame — `"aruco_marker"`, `"cup_holder"`, and/or `"hole"` class entries all share this one array/topic. |

Published continuously, every processed camera frame, plain reliable QoS
(not `transient_local`) — a derived detection stream consumers (e.g.
`depth_perception`'s planned hole/cupholder pipeline,
`CalibrationOrchestratorNode`'s image-based centering) filter/vote over
across multiple frames on their own side, not a one-shot/on-demand sample.

**Detection2D**

| Field | Type | Meaning |
|---|---|---|
| `class_name` | `string` | `"aruco_marker"`, `"cup_holder"`, or `"hole"` — a plain string (not an enum) so a class rename/addition on the YOLO side doesn't require a message-shape change. |
| `cx`, `cy` | `float64` | Pixel coordinates of the detection's centroid (bbox center) in the source image. |
| `confidence` | `float64` | Detection confidence; `1.0` for `"aruco_marker"` (neither detector has a meaningful per-marker confidence score for that class). |
| `bbox` | `float64[4]` | `[x1, y1, x2, y2]` in pixels — lets a consumer (e.g. depth lookup) sample a small patch rather than one noisy pixel. |
| `hole_number` | `int32` | Only meaningful for `class_name == "hole"` — a fixed image-space quadrant label (1=top-left, 2=top-right, 3=bottom-left, 4=bottom-right), computed from each hole's centroid relative to the frame's cupholder/hole-group center. `0` (unset) for `"aruco_marker"`/`"cup_holder"`, since only one of each ever exists in frame. |

`cup_holder`/`hole` are 2D pixel space only, never a 3D pose from this
message — unlike the ArUco marker (known 45 mm real-world size, solvable
via `solvePnP`), there is no known real-world circle size to solve a 3D
pose against from YOLO alone; that's left to `depth_perception`'s own
downstream pipeline (back-projecting via depth + camera intrinsics).

---

## StablePosition.msg / StablePositionArray.msg

Published by `depth_perception_node` on
`/depth_perception/stable_positions`, every processed `detections_2d`
callback — one `StablePosition` per tracked instance (`cup_holder`, or
each `hole_1..hole_4`) **ever** successfully detected so far, regardless
of whether that instance appeared in the frame that triggered this
publish. Unlike `Detection2DArray` (raw, per-frame, unfiltered), this is
`depth_perception_node`'s held/filtered output — a continuous, gap-free
stream a consumer can always rely on having a usable position from,
instead of handling "this instance vanished for a few frames" itself. See
[depth_perception.md](../depth_perception.md) for the rolling-window +
hold-last-known-position filtering that produces it.

**StablePositionArray**

| Field | Type | Meaning |
|---|---|---|
| `header` | `std_msgs/Header` | Same convention as `Detection2DArray`/`marker_pose`: reuses the source image's own header. |
| `positions` | `StablePosition[]` | One entry per tracked instance ever seen. |

**StablePosition**

| Field | Type | Meaning |
|---|---|---|
| `class_name` | `string` | `"cup_holder"` or `"hole"`. |
| `hole_number` | `int32` | Meaningful only for `"hole"` (1–4); `0` for `"cup_holder"`. |
| `x`, `y`, `z` | `float64` | Held 3D position, camera's own optical frame (meters) — `depth_perception_node`'s `last_stable`, not a single frame's raw back-projection. |
| `px`, `py` | `float64` | The same position reprojected back to 2D pixel coordinates, so a consumer that only wants to draw it never needs its own camera-frame math. |
| `drifted` | `bool` | `true` if this message's position was just updated (a genuine drift past `stable_drift_threshold_m`), `false` if still holding a previously-established position. |
| `sample_count` | `int32` | Number of samples currently in this instance's rolling window — a rough confidence/maturity signal only. |

---

## PlanningFailure.msg

Published once, on `trajectory_planner`'s `~/planning_failure` topic, at
the moment a commanded move fails (startup home move, `cal_ready` button,
sequenced-goal lift/standby moves, etc.) — an event, not a state, so plain
reliable QoS, not `transient_local`: a failure that already happened has no
meaningful "current value" to replay to a late subscriber.

| Field | Type | Meaning |
|---|---|---|
| `context` | `string` | Short machine-readable label for which operation failed, e.g. `"startup_home"`, `"cal_ready"`, `"return_to_previous"`, `"standby"` — a plain string, not an enum, so new failure sources don't require a message change. |
| `message` | `string` | Human-readable failure reason, safe to show directly in a popup. |

Intended consumer: the web app shows a popup using `message`; the terminal
already gets the matching `RCLCPP_ERROR` independently.
