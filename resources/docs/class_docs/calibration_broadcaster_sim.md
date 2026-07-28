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
| `orientation_markley_priority` | int | `0` | Priority for Markley's method, a more rigorous SO(3) average robust to widely spread samples. Left at `0` (disabled) because this method isn't implemented yet — see `orientation_averaging.hpp`. |

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
