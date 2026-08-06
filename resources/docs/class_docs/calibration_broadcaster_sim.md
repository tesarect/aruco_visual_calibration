[← Back to index](./README.md)

# calibration_broadcaster_sim.yaml — parameter reference

Parameters for `calibration_broadcaster_node`, loaded under its
`ros__parameters` namespace. See [aruco_perception.md](./aruco_perception.md)
for `CalibrationBroadcasterNode` itself.

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `marker_pose_topic` | string | `/aruco_perception/marker_pose` | Topic to subscribe to for the detector's camera → marker pose. |
| `known_chain_frame` | string | `base_link` | The TF frame this node solves for the camera relative to — the "known" end of the known chain (`known_chain_frame → marker_frame`), fully determined by the arm's joint states in sim since both marker and camera ride the wrist. |
| `marker_frame` | string | `rg2_gripper_aruco_link` | The TF frame of the physical marker mounted on the end effector — the other end of the known chain. |
| `broadcast_frame_suffix` | string | `_calibrated` | Appended to the detector's camera `frame_id` to form the broadcast TF's `child_frame_id` (e.g. `wrist_rgbd_camera_depth_optical_frame` → `wrist_rgbd_camera_depth_optical_frame_calibrated`). Required so the computed result never collides with the URDF-declared frame of the same base name already in the TF tree (sim's ground-truth camera frame, in this case). |
| `num_samples` | int | `10` | How many marker detections to collect and average during the **polygon phase** (position arithmetically, orientation via the configured averaging method), cycling through the polygon waypoints for up to 2 passes. Fewer than this if early-stop triggers first. |
| `sample_wait_timeout_sec` | double | `5.0` | How long to wait for a *fresh* `marker_pose` message (stamped after the arm's settle point) after each move, before aborting that sample — and the whole calibration run — e.g. if the marker goes out of view. |
| `planning_mode` | string (enum) | `cartesian` | Planning strategy requested on every `~/trace_path` call this node makes. One of `cartesian` (straight-line, can fail partway near limits/obstacles) or `joint_space` (free-space, more robust, no straight-line guarantee) — see `TracePath.srv`. |
| `orientation_sum_normalize_priority` | int | `1` | Priority for the "sum and renormalize" quaternion-averaging method (lower positive number = tried first, `0` = disabled). Correct enough when samples are already close together, which holds here since all samples are of the same physical marker/camera pair. |
| `orientation_markley_priority` | int | `0` | Priority for Markley's method, a more rigorous SO(3) average robust to widely spread samples (implemented — see `orientation_averaging.hpp`). Left at `0` (disabled) as an opt-in, not a default-behavior change. |

## Random phase

Runs after the polygon phase completes (or is skipped, if early-stop
already triggered during the polygon phase) — see
[aruco_perception.md](./aruco_perception.md#calibrationbroadcasternode) for
the two-phase design.

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `random_phase_samples` | int | `8` | Number of additional samples to collect at randomized offsets from the polygon phase's center pose, after the polygon phase completes. |
| `random_phase_max_offset_m` | double | `0.10` | Maximum straight-line distance (meters) a random candidate pose may be from the center pose, checked before moving there. |
| `random_phase_max_consecutive_failures` | int | `20` | Safety bound: consecutive discarded candidates (move succeeded but marker not visible) allowed before the random phase gives up and aborts the whole calibration run. Not expected to be hit in normal operation. |

## Early-stop

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `position_spread_tolerance_cm` | double | `2.0` | A sample's position is "in agreement" if it's within this distance of the running mean of all samples collected so far. Both this and `orientation_spread_tolerance_deg` must hold for a sample to count toward `stable_agreement_count`. |
| `orientation_spread_tolerance_deg` | double | `5.0` | Angular equivalent of `position_spread_tolerance_cm`, checked against the running orientation average. |
| `stable_agreement_count` | int | `2` | Number of samples (not necessarily consecutive) that must fall within both spread tolerances of the running average before collection stops early and proceeds straight to averaging/broadcasting, instead of always running the full polygon+random count. |

## Orientation sweep phase

| Parameter | Type | Default (sim) | Meaning |
|---|---|---|---|
| `orientation_sweep_enabled` | bool | `false` | Runs `runOrientationSweepPhase` once after polygon/random sampling — 4 rotational probes (pitch/roll) from `cal_ready`'s orientation. Disabled on sim (ground-truth camera TF makes it largely redundant); default true on real, added after a run showed poor orientation diversity from position-only sampling alone. |
| `orientation_sweep_angle_deg` | double | `5.0` | Offset magnitude (degrees) for all 4 sweep probes. |

## Outlier rejection, dual-sampling, clustering, yaw/roll clamp

See [aruco_perception.md](./aruco_perception.md#calibrationbroadcasternode)'s
"Outlier rejection" / "Clustering-based averaging" / "Yaw/roll clamp"
sections for the underlying algorithms — summarized here as parameters:

| Parameter | Type | Default (sim) | Meaning |
|---|---|---|---|
| `outlier_rejection_enabled` | bool | `true` | Discards samples deviating from the median position/medoid orientation beyond the thresholds below, before averaging. |
| `outlier_position_threshold_cm` | double | `2.0` | Position deviation threshold for outlier rejection. |
| `outlier_orientation_threshold_deg` | double | `5.0` | Orientation deviation threshold for outlier rejection. |
| `samples_per_waypoint` | int | `2` | Samples taken at each settled waypoint before advancing (no additional move between them) — mitigates one bad/missed detection. Set to `1` to restore one-sample-per-waypoint. |
| `use_clustering_average` | bool | `false` | Live-toggled (not restart-only): uses `computeClusteredPose` (largest agreeing position+orientation cluster) instead of a plain mean, when true. |
| `clustering_bucket_size_cm` | double | `2.0` | Position tolerance for two samples to join the same cluster. |
| `clustering_bucket_angle_deg` | double | `5.0` | Orientation tolerance for two samples to join the same cluster. |
| `yaw_roll_clamp_enabled` | bool | `false` | Real-only hypothesis test: replaces every sample's yaw/roll with a run-wide circular mean before averaging, on the theory that yaw/roll variation on a physically-fixed camera mount is corner-detection noise. Kept off on sim — sim's ground-truth TF already lets accuracy be verified directly. |

Per-waypoint hybrid detection (`hybrid_per_waypoint_enabled`,
`detect_call_timeout_sec`, `min_samples_to_finish`,
`cal_ready_hybrid_marker_detection_retry`) is real-only in practice — not
wired into `calibration_broadcaster_sim.yaml` at all, since sim's
classical detector has no real-world corner-detection noise to address
with it. See `calibration_broadcaster_real.yaml`'s own comments for those
parameters.
