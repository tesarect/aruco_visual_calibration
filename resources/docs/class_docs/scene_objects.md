[← Back to index](./README.md)

# scene_objects.yaml — parameter reference

Parameters for `planning_scene_setup`, loaded under its `ros__parameters`
namespace, covering both `scene_objects_sim.yaml` and
`scene_objects_real.yaml`. See
[visual_calibration_moveit.md](./visual_calibration_moveit.md) for
`PlanningSceneSetup` and the `SceneObjectConfig` types these parameters are
read into.

**Real-robot values are a placeholder.** `scene_objects_real.yaml` is not
measured on the real robot yet — its values are currently identical to
sim's. The real lab layout (Barista body, table, wall-to-camera/arm
distances) is visually similar to sim but **not identical** — do not treat
any value below as correct just because it currently matches sim. All
poses/dimensions must be measured/verified against the real cell before
use.

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `planning_frame` | string | `base_link` (both) | The TF frame every object pose below is expressed in. |

Each of the four known cafeteria obstacles (`coffee_machine`, `cupholder`,
`countertop`, `wall`) is declared as a flat group of `<object>.<field>`
parameters rather than one parameter per field name, since MoveIt2's
`ros__parameters` YAML has no native nested-list-of-objects support. Sim
and real currently carry identical values for every object below (real is
an as-yet-unverified copy of sim), so one column suffices; if the two
diverge in the future, note both variants explicitly per row the way
`trajectory_planner.md` does.

## coffee_machine / cupholder (mesh objects)

| Parameter | Type | Default (coffee_machine / cupholder) | Meaning |
|---|---|---|---|
| `<name>.shape_type` | string (enum) | `mesh` / `mesh` | Which collision-geometry kind this object uses — `mesh` (loaded `.dae` file) or `box` (see countertop/wall below). |
| `<name>.pose.x` | double | `0.1` / `-0.26` | Object origin X (meters), in `planning_frame`. |
| `<name>.pose.y` | double | `0.86` / `0.04` | Object origin Y (meters). |
| `<name>.pose.z` | double | `-0.032` / `-0.632` | Object origin Z (meters). |
| `<name>.pose.yaw` | double | `1.57` / `1.57` | Object yaw (radians) about Z — no full quaternion, since every known object only needs yaw. |
| `<name>.mesh_path` | string | cafeteria/barista `.dae` package path (see live YAML) | `package://`-style path to the collision mesh, sourced from `the_construct_office_gazebo`'s models. |

## countertop / wall (box objects)

Modeled as one-or-more axis-aligned boxes instead of a mesh — the countertop
(`starbots_bartender_dispenser` in Gazebo) as two stacked boxes (body + thin
top slab), the wall as a single box.

| Parameter | Type | Default (countertop / wall) | Meaning |
|---|---|---|---|
| `<name>.shape_type` | string (enum) | `box` / `box` | Collision-geometry kind — `box` here. |
| `<name>.pose.x` | double | `0.3` / `0.3` | Object base pose X (meters). |
| `<name>.pose.y` | double | `0.36` / `-0.56` | Object base pose Y (meters). |
| `<name>.pose.z` | double | `-0.532` / `-0.032` | Object base pose Z (meters). |
| `<name>.pose.yaw` | double | `0.0` / `0.0` | Object base pose yaw (radians). |
| `<name>.box_names` | string[] | `["body", "top"]` / `["body"]` | Names of the sub-boxes making up this object; each name below gets its own `<name>.boxes.<box_name>.*` entry. |
| `<name>.boxes.<box_name>.size` | double[3] | e.g. `countertop.boxes.body.size: [0.5, 1.8, 1.0]` | Box dimensions (x, y, z, meters). |
| `<name>.boxes.<box_name>.local_pose` | double[4] | e.g. `countertop.boxes.top.local_pose: [0.0, 0.0, 0.5, 0.0]` | This sub-box's own pose offset (x, y, z, yaw) from the parent object's base pose — used so e.g. the countertop's top slab can sit above its body without a second top-level object. |
