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
| `polygon_num_corners` | int | `4` (both — real copied from sim, not yet re-tuned) | Number of waypoints placed evenly around the arm's current-pose local X/Y plane for calibration-sample spread via `~/trace_polygon` / `~/get_polygon_waypoints`. Must be ≥ 3 (3 = triangle, 4 = square, 5 = pentagon, ...). |
| `polygon_radius_m` | double | `0.05` (both — real copied from sim, not yet re-tuned) | Each polygon corner's distance (meters) from the center pose, in its own local X/Y plane. Should be kept small relative to `max_reach_m` so every corner stays reachable. |
| `polygon_default_planning_mode` | string (enum) | `cartesian` (both) | Default planning strategy for `~/trace_polygon` (a plain `Trigger`, with no per-call field to override it). One of `cartesian` (straight-line interpolated path; predictable geometry but can fail partway near limits/obstacles) or `joint_space` (free-space planning; more robust, no straight-line guarantee). `~/trace_path` takes `planning_mode` explicitly per call instead — see `TracePath.srv`. |

## Planner tuning (`PlannerConfig`)

Applied to every joint-space `planAndExecute()` call (both the pose and
joint-value overloads); `planAndExecuteCartesian()` uses
`computeCartesianPath()` instead and only reads `cartesian_min_fraction`
from this group.

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `planning_pipeline_id` | string | `ompl` (both) | Must stay `"ompl"` — `move_group.launch.py` deliberately restricts `move_group` to only load the OMPL pipeline. CHOMP/Pilz are neither configured nor tested against this project's planning group; changing this value alone, without also authoring that pipeline's yaml and updating `move_group.launch.py`, does nothing useful. |
| `planner_id` | string | `RRTstarkConfigDefault` (both) | Must be one of `ur_manipulator`'s `planner_configs` entries in `{sim,real}_ur3e_moveit_config/config/ompl_planning.yaml`. An optimizing planner (keeps searching until `planning_time_s` runs out, minimizing joint-space path length) — chosen after the previously-unconfigured default (MoveIt's compiled-in RRTConnect, 1 attempt, no optimization) was found to accept the first valid-but-twisted path with nothing pushing toward a shorter/cleaner one. |
| `planning_time_s` | double | `3.0` (sim) | Seconds allotted per planning attempt. |
| `num_planning_attempts` | int | `3` (sim) | `MoveGroupInterface` automatically keeps the shortest-path plan among this many attempts. |
| `cartesian_min_fraction` | double | sim: `0.75`, real: see file (both a **safety tradeoff**, not a pure tuning knob) | Minimum achieved fraction of a straight-line Cartesian path required to execute it at all (`planAndExecuteCartesian`'s `min_fraction`). Sim's value was lowered from the original 0.95 default after `calibration_orchestrator_node`'s image-based centering probes (small ~0.05–0.10 m moves near `cal_ready`) were found to legitimately achieve only 50–83% on some directions near that pose — a real geometric limitation of the straight-line path from that specific start, not a bug. Lowering this means MORE incomplete paths get executed rather than refused, i.e. the arm can stop short of the intended waypoint at an unplanned intermediate pose — only lower it if that tradeoff is deliberately acceptable. |
| `planning_time_retry_multipliers` | double[] | sim: `[]` (no retries — original single-attempt behavior), real: `[3.0, 5.0]` | Escalating-retry budget for joint-space planning (`planWithEscalatingTime`): each retry multiplies `planning_time_s` by the next entry (e.g. real's `3.0s` base × `[3.0, 5.0]` = `3s`, then `9s`, then `15s` attempts). Motivated by a real-robot OMPL log showing a genuine search timeout (not a hard reachability/collision failure) for `~/move_to_instance`'s hover-pose leg — a longer budget is worth trying for a target near the edge of what `RRTstar` can solve quickly before concluding it's genuinely unreachable. |

## `~/move_to_instance` hover/descend tuning (`HoverConfig`)

Only applies to `~/move_to_instance` (see
[visual_calibration_moveit.md](./visual_calibration_moveit.md)'s
`~/move_to_instance` section for the full 5-step sequence).

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `instance_hover_offset_m` | double | sim: `0.20`, real: `0.40` | Height (meters) above `cup_holder`'s own TF the shared hover/approach pose targets. |
| `instance_descend_offset_m` | double | `0.15` (both) | How far (meters) the descend leg travels down from the hover pose's Z toward the requested instance's own Z — independent of `instance_hover_offset_m`, so the final approach distance isn't tied to how high the hover point sits. Not clamped against the hover offset — a larger descend offset than the hover offset would overshoot below the instance's own Z. |
| `instance_stay_seconds` | double | `3.0` (both) | How long to stay at the descended goal before returning to the hover pose. |
| `instance_return_preset_name` | string | `""` (both) | No longer used by `handleMoveToInstance` (superseded by an unconditional lift-and-wait step) — left in config only for a possible future caller. |
| `reach_safety_margin_m` | double | sim: `0.08`, real: `0.05` | Safety margin subtracted from `max_reach_m` to get the descend leg's own clamp radius. If the raw descend pose would land farther than `max_reach_m - reach_safety_margin_m` from the planning frame's origin, it's pulled back along the hover→descend line to that radius instead of failing outright — the arm still visibly approaches the target and stops at the closest safely-reachable point. `0.0` disables clamping. |

## Sequenced-goal timing (`SequenceConfig`)

`stay_seconds_at_goal`/`lift_wait_seconds` only apply to `~/trace_path`
calls that explicitly set `is_sequenced_goal: true` (see `TracePath.srv` and
`ArmState`, in [visual_calibration_moveit.md](./visual_calibration_moveit.md)).
`lift_target_z_m` is also reused directly by `~/move_to_instance`'s own
final lift step (independent of the `is_sequenced_goal`/`ArmState`
machinery — see that section of `visual_calibration_moveit.md`).

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `stay_seconds_at_goal` | double | `4.0` (both) | How long to stay AT a sequenced goal (e.g. a hole/cupholder pose) before automatically lifting away from it. |
| `lift_target_z_m` | double | `0.0` (both) | Absolute Z (in the planning frame — `base_link`'s own Z) to lift to after `stay_seconds_at_goal` — NOT an offset added to the goal's Z. X/Y/orientation stay identical to the goal. Also the target Z for `~/move_to_instance`'s own final lift step. |
| `lift_wait_seconds` | double | `8.0` (both) | How long to then wait at the lifted pose, with no new sequenced goal arriving, before automatically moving to the `"standby"` preset. |
| `waypoint_settle_seconds` | double | `1.0` (both) | Global delay applied inside `tracePath()` after EVERY successful waypoint move (single or multi-waypoint calls alike — home, cal_ready, sequenced goals, and each polygon corner all funnel through `tracePath()`). Gives a real camera time to produce a fresh, non-motion-blurred frame after the arm stops moving — sim can visually keep up instantly, but a slower real camera pipeline may not. `0.0` disables it. |
| `gripper_close_settle_seconds` | double | `2.0` (both) | How long to pause after publishing the startup gripper-close command (see `closeGripperOnStartup`) before `runStartupSequence` proceeds to the home move. No completion feedback exists to wait on instead — a fixed guess, not a measurement. Runs on sim too even though nothing there subscribes to `/gripper/cmd` (the publish is a no-op) — the pause still happens either way. |

## Startup behavior

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `move_to_home_on_startup` | bool | `true` (both) | If true, `TrajectoryPlanner` moves to the `"home"` preset once on startup (see `runStartupSequence`, called from the constructor) — requires a `"home"` entry in `preset_poses_{sim,real}.yaml`. An explicit, opt-in reversal of this node's original "never move on startup" design. |
