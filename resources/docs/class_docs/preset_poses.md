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

`home`, `standby`, `baristastandby`, and `cal_ready` (== `standoff`, same
pose under two names) are all Cartesian, real-measured — no joint-value
preset has been captured for real yet, so `~/move_to_preset("cal_ready")`
there falls back to the Cartesian preset. `standoff` itself is not a fixed
preset entry consulted by `~/move_to_preset` (that method never touches
TF) — it exists as `getStandoffPose()`'s fallback source when the live
`camera_frame` TF lookup fails.
