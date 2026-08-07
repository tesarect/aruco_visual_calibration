[← Back to index](./README.md)

# preset_poses_{sim,real}.yaml — parameter reference

Parameters for `trajectory_planner`, loaded into that node's own parameter
namespace (not a separate node — passed as a second `parameters` entry
alongside `trajectory_planner_{sim,real}.yaml`). See
[visual_calibration_moveit.md](./visual_calibration_moveit.md) for
`PresetPoses` itself.

| Parameter | Type | Meaning |
|---|---|---|
| `preset_names` | string[] | List of preset names this file defines — each name below must have EITHER a `<name>.position`/`<name>.orientation` pair OR a `<name>.joint_values` array (never both). |
| `<name>.position` | double[3] | Cartesian position (x, y, z, meters) for `end_effector_frame`, in the planning frame — same convention `~/trace_path`'s waypoints use. |
| `<name>.orientation` | double[4] | Cartesian orientation (x, y, z, w quaternion). |
| `<name>.joint_values` | double[6] | Joint-space preset instead of Cartesian — 6 values in `ur_manipulator`'s own joint order (shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3), radians. Pins the exact IK branch — see `PresetPoses`' doc comment for why this exists alongside Cartesian presets. |

Cartesian presets are recorded via `pose_capture.py`; joint-value presets
via `ros2 topic echo /joint_states --once` at the desired pose (see
`resources/scripts/python/`).

## Sim (`preset_poses_sim.yaml`)

| Preset | Kind | Used by |
|---|---|---|
| `home` | Cartesian | `move_to_home_on_startup` (auto-move at node construction) — sim's arm spawn pose, recorded as a preset rather than relying on wherever the arm happened to spawn. |
| `standby` | Cartesian | The sequenced-goal idle-timeout destination (see `ArmState::LIFTED_IDLE` → `STANDBY`). |
| `baristastandby` | Cartesian | An additional named stop, not part of the automatic sequence. |
| `cal_ready` | **Joint-value** | `~/move_to_preset("cal_ready")`, preferred by `calibration_orchestrator_node`'s Stage 1 over the TF-derived standoff pose. Captured after confirming this specific joint configuration lets the full calibration polygon sweep succeed, unlike a direct move to the same Cartesian target, which consistently failed the first polygon corner's Cartesian interpolation partway (a different IK branch, ~29° vs ~21° of margin off the `wrist_1` limit). |

## Real (`preset_poses_real.yaml`)

| Preset | Kind | Used by |
|---|---|---|
| `home` | Cartesian | `move_to_home_on_startup` (auto-move at node construction). |
| `standoff` | Cartesian | `getStandoffPose()`'s fallback source when the live `camera_frame` TF lookup fails — not a fixed preset `~/move_to_preset` itself resolves to (that method never touches TF). Position/orientation match `cal_ready` (recorded independently, at the same physical pose, under two names). |
| `cal_ready` | **Joint-value** (re-measured 2026-07-23) | `~/move_to_preset("cal_ready")`, same role as sim's — captured by manually jogging to a pose where the marker was confirmed cleanly, stably visible to the wall-mounted D415 (the previous Cartesian-only `cal_ready`/`standoff` pose lost the marker within a second of arriving), then reading back the exact joint values via `pose_capture.py`. Not yet verified against a full live `~/auto_calibrate` run — only marker visibility/stability at the pose itself was confirmed as of this writing. |
| `cal_ready_alt` / `cal_ready_alt2` | Joint-value, **untested** | Two alternate joint configurations reaching essentially the same Cartesian pose as `cal_ready` via different UR3e IK branches (captured 2026-07-20, before `cal_ready` itself became a joint-value preset) — kept under separate names specifically because neither has been verified against a live calibration run, unlike sim's `cal_ready.joint_values`. Not part of `~/move_to_preset("cal_ready")`'s default resolution; reachable only by explicitly requesting `cal_ready_alt`/`cal_ready_alt2` by name. |
| `standby` | Cartesian | The sequenced-goal idle-timeout destination. |
| `baristastandby` | Cartesian | An additional named stop, not part of the automatic sequence. |
