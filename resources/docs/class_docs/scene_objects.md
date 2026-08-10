[← Back to index](./README.md)

# scene_objects.yaml — parameter reference

Parameters for `planning_scene_setup`, loaded under its `ros__parameters`
namespace, covering both `scene_objects_sim.yaml` and
`scene_objects_real.yaml`. See
[visual_calibration_moveit.md](./visual_calibration_moveit.md) for
`PlanningSceneSetup` and the `SceneObjectConfig` types these parameters are
read into.

**Sim and real are no longer identical.** Real's file was rewritten
following a real-robot collision incident during manual jogging (RViz
collision display was off) — every real object is now a deliberately
oversized, roughly-estimated placeholder box (never a mesh, and never
smaller/further than the true object), to give some immediate collision
protection while accurate measurements are pending via the
`measure_scene_box.py` / `measurecorner`+`measurecompute` workflow. Do not
assume any real value below is precise, and do not assume sim and real
values match for the same object.

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `planning_frame` | string | `base_link` (both) | The TF frame every object pose below is expressed in. |
| `<object>.enabled` | bool | `true` (both, all objects) | Per-object on/off switch — set `false` to remove that object from the scene without deleting its config. Defaults `true` if omitted. |

Each known obstacle is declared as a flat group of `<object>.<field>`
parameters rather than one parameter per field name, since MoveIt2's
`ros__parameters` YAML has no native nested-list-of-objects support.

## Sim (`scene_objects_sim.yaml`)

| Object | Shape | Notes |
|---|---|---|
| `coffee_machine` | mesh | Unchanged — `package://the_construct_office_gazebo/models/coffee_machine/meshes/cafeteria.dae`. |
| `cupholder` | box (was mesh) | Switched from mesh to box after the instructor's updated `the_construct_office_gazebo` package removed the mesh directory the old path resolved to — not yet re-measured against the new sim world, a placeholder box. |
| `countertop` | box, 2 stacked sub-boxes (`body` + `top`) | Unchanged. |
| `wall` | box | Unchanged. |
| `camera` | box (new) | Sim's camera is wrist-mounted (rides the arm, part of the URDF) — there is no physical wall-mounted camera housing to guard against, unlike real. Kept enabled anyway as a placeholder/visual-reference object, matching real's `camera.*` entries in shape for style parity, not because it represents a real collision concern in sim. |

## Real (`scene_objects_real.yaml`)

`coffee_machine`, `cupholder`, `countertop`, `wall`, and `camera` are
currently deliberately oversized generic boxes at rough estimated
positions — not measured, and specifically NOT the old sim-copied
mesh/box values that were here before (wrong shape at a wrong position
gave false confidence). `camera` (added after the incident) is an
unmeasured placeholder guarding against the wall-mounted D415 protruding
into the arm's path.

Two further box objects were added afterward, both from live-jogged
positions rather than rough estimates:

- `table_edge_guard` — a long box laid along the countertop's edge closest
  to the arm's own `base_link` origin, resting on the table surface;
  position/rotation were directly jogged and read back from RViz.
- `base_slab` — a thin box directly below `base_link`, guarding against
  the shoulder/upper-arm dipping down and clipping the table surface
  right under the arm's own mount point (widening `countertop`'s own box
  did not cover this, since the shoulder passes close to `base_link`'s
  own origin, not necessarily within `countertop`'s box extent) — a rough
  starting value, not yet confirmed against the real table height.

Fields per object, same structure on both sim and real:

| Parameter | Type | Meaning |
|---|---|---|
| `<name>.shape_type` | string (enum) | `mesh` (loaded `.dae` file) or `box`. |
| `<name>.pose.x` / `.y` / `.z` | double | Object base pose (meters), in `planning_frame`. |
| `<name>.pose.yaw` | double | Object base pose yaw (radians) about Z — no full quaternion, since every known object only needs yaw. |
| `<name>.mesh_path` | string | `package://`-style path to the collision mesh — only for `shape_type: "mesh"`. |
| `<name>.box_names` | string[] | Names of the sub-boxes making up this object — only for `shape_type: "box"`; each name gets its own `<name>.boxes.<box_name>.*` entry (most objects use a single `["body"]`; `countertop` uses `["body", "top"]`). |
| `<name>.boxes.<box_name>.size` | double[3] | Box dimensions (x, y, z, meters). |
| `<name>.boxes.<box_name>.local_pose` | double[4] | This sub-box's own pose offset (x, y, z, yaw) from the parent object's base pose — used so e.g. the countertop's top slab can sit above its body without a second top-level object. |

See the live YAML files for current numeric values — real's in particular
should be treated as actively in flux until the measurement pass is done.
