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
| `num_samples` | int | `10` | How many marker detections to collect and average (position arithmetically, orientation via the configured averaging method) before broadcasting the final static TF. Samples are spread across waypoints (see `trajectory_planner.md`'s polygon parameters) rather than taken repeatedly from one fixed pose. |
| `sample_wait_timeout_sec` | double | `5.0` | How long to wait for a *fresh* `marker_pose` message (stamped after the arm's settle point) after each move, before aborting that sample — and the whole calibration run — e.g. if the marker goes out of view. |
| `planning_mode` | string (enum) | `cartesian` | Planning strategy requested on every `~/trace_path` call this node makes. One of `cartesian` (straight-line, can fail partway near limits/obstacles) or `joint_space` (free-space, more robust, no straight-line guarantee) — see `TracePath.srv`. |
| `orientation_sum_normalize_priority` | int | `1` | Priority for the "sum and renormalize" quaternion-averaging method (lower positive number = tried first, `0` = disabled). Correct enough when samples are already close together, which holds here since all samples are of the same physical marker/camera pair. |
| `orientation_markley_priority` | int | `0` | Priority for Markley's method, a more rigorous SO(3) average robust to widely spread samples. Left at `0` (disabled) because this method isn't implemented yet — see `orientation_averaging.hpp`. |
