[← Back to index](../README.md)

# visual_calibration_msgs — interface docs

Interfaces-only package — no C++ classes, just `.srv`/`.action`
definitions shared between `aruco_perception` and `visual_calibration_moveit`.
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

The computed `known_chain_frame → camera` transform itself isn't included
here — it's already broadcast on `/tf` by the time the action completes;
read it from there.

**Feedback**

| Field | Type | Meaning |
|---|---|---|
| `samples_collected` | `uint32` | How many samples have been accepted so far. |
| `samples_total` | `uint32` | Total samples this run will collect. |

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
Computes and returns the polygon waypoints around the standoff pose
**without moving the arm**, so a caller (e.g.
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
