[← Back to index](./README.md)

# sim_ur3e_moveit_config / real_ur3e_moveit_config

`sim_ur3e_moveit_config` and `real_ur3e_moveit_config` are project-owned,
environment-split copies of the instructor-provided
`universal_robot_ros2/ur3e_moveit_config`. They are pure MoveIt2
configuration (SRDF, YAML, launch files) with no hand-written classes — no
`class_docs/` entry exists for either, see
[class_docs/README.md](./class_docs/README.md)'s own skip note. `move_group`
for both sim and real launches from one of these two packages today, not
from the untouched `universal_robot_ros2/ur3e_moveit_config` (never edited
directly — see this project's `CLAUDE.md`).

## Why two copies exist instead of one

`universal_robot_ros2/` is the instructor-provided base environment and is
never edited directly (see this project's `CLAUDE.md`). Its
`ur3e_moveit_config` is tuned for the **real** robot's controllers. Running
that same config against Gazebo sim fails: sim's `ros2_control` setup
exposes plain `joint_trajectory_controller`, while the real robot's exposes
`scaled_joint_trajectory_controller` — using the wrong controller name for
the wrong environment makes MoveIt accept a plan but abort execution, since
it dispatches to an action server that doesn't exist in that environment.
Rather than edit the instructor's package in place (forbidden) or maintain
a single config with runtime branching, each environment gets its own full
copy, following the same pattern already used elsewhere in this project
(e.g. `trajectory_planner_sim.yaml` vs. `_real.yaml`).

## What actually differs between the two packages

Both are near-identical copies; the two real differences are:

- **`config/moveit_controllers.yaml`** — sim's controller list is
  `joint_trajectory_controller` / `gripper_controller` (matching Gazebo's
  actual ros2_control setup, RG2 gripper joint); real's is
  `scaled_joint_trajectory_controller` / the Robotiq 85 gripper's
  `robotiq_85_left_knuckle_joint`. This is the one file that differs from
  the instructor's original real-robot config.
- **`launch/move_group.launch.py`** — sim's applies `use_sim_time: true` to
  every node in the launch group (`SetParameter` inside a `GroupAction`);
  real's does not set it at all, since real has no `/clock` and must stay
  on wall time. Gazebo publishes `/joint_states` timestamped against sim
  time; without this, `move_group`'s current-state monitor compares
  incoming timestamps against wall-clock "now" and treats every message as
  stale no matter how fast Gazebo actually publishes — planning succeeds
  (it doesn't need live state) but execution always aborts (it validates
  against current state first).

## Shared fixes present in both launch files

Both `move_group.launch.py` files carry the same two non-obvious fixes,
found while getting either environment working and applied identically to
both once found:

- **`.planning_pipelines(pipelines=["ompl"])` is called explicitly**, and
  both packages deliberately ship only `ompl_planning.yaml` — no
  `chomp_planning.yaml` or Pilz planning config exists, and neither
  planner has been configured or tested against this project's planning
  group. Restricting the pipeline list is necessary but **not sufficient**
  on its own to keep CHOMP out of the process (see next point).
- **`capabilities` is explicitly whitelisted** to only the
  `MoveGroupCapability` names `TrajectoryPlanner` actually uses (no
  `/plan_kinematic_path` call exists anywhere in this codebase). Restricting
  the planning pipeline alone was confirmed insufficient: `move_group`
  loads the CHOMP planner plugin directly via `pluginlib` as part of one of
  its *default* capabilities (most likely `MoveGroupQueryPlannersService`),
  completely independent of the `planning_pipelines` config — confirmed by
  inspecting `move_group`'s own process memory maps and finding
  `libmoveit_chomp_planner_plugin.so` loaded even with an OMPL-only
  pipeline list. `moveit-planners-chomp` is installed system-wide in this
  project's environment (never removed — that would touch the shared base
  environment), so its plugin is globally discoverable by `pluginlib`
  regardless of this project's own yaml. Explicitly whitelisting
  capabilities is what actually keeps it from loading.
- **`.trajectory_execution(file_path="config/moveit_controllers.yaml")`
  and `.pilz_cartesian_limits()` are both called explicitly**, rather than
  relying on `MoveItConfigsBuilder`'s auto-discovery fallback — once
  `.planning_pipelines()` is called explicitly, the builder's
  `to_moveit_configs()` no longer auto-invokes the other builder steps as a
  fallback, and omitting `.pilz_cartesian_limits()` entirely crashes
  `move_group` startup outright (`KeyError: 'robot_description_planning'`,
  from `moveit_configs_builder.py`'s `to_dict()` unconditionally reading
  that key with no `None` check).
- **`trajectory_execution.allowed_execution_duration_scaling` is
  overridden to `4.0`** via `SetParameter`, rather than disabling the
  execution-duration watchdog outright (`execution_duration_monitoring:
  false` was tried and abandoned) — the watchdog stays active, just with
  more time budget, after some real trajectories were found to legitimately
  need it.
