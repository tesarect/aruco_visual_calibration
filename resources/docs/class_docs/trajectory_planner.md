[← Back to index](./README.md)

# trajectory_planner.yaml — parameter reference

Parameters for the `trajectory_planner` node, loaded under its
`ros__parameters` namespace, covering both `trajectory_planner_sim.yaml`
and `trajectory_planner_real.yaml`. See
[visual_calibration_moveit.md](./visual_calibration_moveit.md) for
`TrajectoryPlanner` itself.

**Real-robot values are a placeholder.** `trajectory_planner_real.yaml` is
not measured or tuned on the real robot yet — aside from `camera_frame`,
its values are simply copied from sim as a starting point and must be
re-verified against the real cell before use; nothing there should be
assumed correct just because it currently matches sim.

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `camera_frame` | string | sim: `wrist_rgbd_camera_depth_optical_frame`, real: `camera_depth_optical_frame` | The TF frame `planAndExecuteInFrontOf` positions the arm relative to — the standoff pose is computed in front of this frame. Differs by design, not just by placeholder: on the real robot the camera isn't arm-mounted, and `wrist_rgbd_camera_depth_optical_frame` doesn't exist there — the real camera's frame is only established once the calibration pipeline itself publishes it, not from a static URDF TF. |
| `end_effector_frame` | string | `rg2_gripper_aruco_link` (both) | The frame that gets moved to the computed standoff/polygon pose (the marker's own frame, not `tool0` — the marker is what needs to face the camera). |
| `standoff_m` | double | `0.25` (both — real copied from sim, not yet re-tuned) | Distance (meters) out along the camera frame's local +Z axis the standoff pose sits. A starting point, not a precise derivation — the camera sits roughly 0.43 m from `base_link` already, close to the UR3e's ~0.5 m reach, so this value was tuned empirically against what actually plans in the cafeteria scene. The real cell's geometry may differ, so this needs re-tuning before real-robot use. |
| `max_reach_m` | double | `0.5` (both) | The UR3e's datasheet maximum reach — used as a reference figure when tuning `standoff_m` and `polygon_radius_m`, not enforced in code as a hard limit. |
| `facing_rpy_rad` | double[3] | `[3.14159265, 0.0, 1.57079633]` (π, 0, π/2) (both — real copied from sim, not yet verified) | Roll/pitch/yaw (radians), applied in the camera frame's own local axes, that rotates the standoff pose so `end_effector_frame` faces back toward the camera. This is a design choice ("how should the marker face the camera"), not something derivable from TF, so it's a tuned parameter rather than a computed value. The default is equivalent to quaternion `(0.7071, 0.7071, 0, 0)`: it swaps the goal's X/Y axes with the camera's and flips Z. **Not verified for the real cell** — the real camera's mounting orientation may not match sim's, so this may need a different rotation; re-derive and verify visually (e.g. via `tf_debug_markers.py`) rather than assuming the sim value carries over. |
| `polygon_num_corners` | int | `4` (both — real copied from sim, not yet re-tuned) | Number of waypoints placed evenly around the standoff pose's local X/Y plane for calibration-sample spread via `~/trace_polygon` / `~/get_polygon_waypoints`. Must be ≥ 3 (3 = triangle, 4 = square, 5 = pentagon, ...). |
| `polygon_radius_m` | double | `0.05` (both — real copied from sim, not yet re-tuned) | Each polygon corner's distance (meters) from the standoff pose's center, in its own local X/Y plane. Should be kept small relative to `max_reach_m` so every corner stays reachable. |
| `polygon_default_planning_mode` | string (enum) | `cartesian` (both) | Default planning strategy for `~/trace_polygon` (a plain `Trigger`, with no per-call field to override it). One of `cartesian` (straight-line interpolated path; predictable geometry but can fail partway near limits/obstacles) or `joint_space` (free-space planning; more robust, no straight-line guarantee). `~/trace_path` takes `planning_mode` explicitly per call instead — see `TracePath.srv`. |
