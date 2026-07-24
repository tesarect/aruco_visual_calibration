# Staged/ordered `bringup_*` launch files

Covers both `bringup_full_sim.{launch.py,xml}` and `bringup_full_real.{launch.py,xml}`
(same directory) — the sim/real split only changes which `env` value is
threaded through; the sequence and gates below are identical either way.

**All files under this naming are new and additive.** Nothing existing
(`planning_scene_setup.launch.py`, `trajectory_planner.launch.py`,
`aruco_detector.launch.py`, `calibration_broadcaster.launch.py`,
`calibration_orchestrator.launch.py`, `yolo_marker_bridge.launch.py`, or any
`*_tmux_*.sh` script) was modified to build this. The tmux-script-driven
setup remains the known-working fallback throughout.

## What's out of scope

Gazebo, `move_group`, RViz, controller-activation (`ensure_controller_active.sh`),
and the Zenoh bridge are **not** started by any of these files — none of them
live in a `visual_calibration/`-owned package. Start the matching base
session (`sim_tmux_base.sh` / `real_tmux_base.sh`) first, same as today,
then run one of the `bringup_full_*` files in place of the corresponding
trajcal tmux session.

## Dependency chain

```
1. Gazebo simulation                         (out of scope — the_construct_office_gazebo)
2. move_group                                (out of scope — sim/real_ur3e_moveit_config;
                                               needs controller_manager's
                                               joint_state_broadcaster ACTIVE)
3. RViz  +  planning_scene_setup             (parallel — both only need move_group up;
                                               RViz out of scope, planning_scene_setup
                                               covered by bringup_moveit_pipeline)
4. trajectory_planner                        (needs move_group + planning scene POPULATED
                                               with {"countertop","wall"} — content check,
                                               not just node-presence)
   -- in parallel with --
   aruco_detector_node                       (needs move_group, ordering-consistency only)
   inference_server.py                       (NOT a ROS node — plain Flask, no rclpy —
                                               needs nothing ROS-side, gated by its own
                                               HTTP /health)
5. calibration_broadcaster_node              (needs aruco_detector_node + trajectory_planner)
   -- in parallel with --
   yolo_marker_bridge_node                   (needs inference_server.py's /health + move_group)
6. calibration_orchestrator_node             (needs calibration_broadcaster_node +
                                               trajectory_planner)                -- LAST --
```

## File map

| File | Package | Combines | Gate it adds (replicates which existing script) |
|---|---|---|---|
| `bringup_moveit_pipeline.launch.py` | `visual_calibration_moveit` | `planning_scene_setup` → `trajectory_planner` | Polls `/get_planning_scene` (service `moveit_msgs/srv/GetPlanningScene`) until collision objects `{"countertop","wall"}` are present — replicates `resources/scripts/python/wait_for_planning_scene.py`'s exact condition, reimplemented inline (that script isn't installed/reachable from an install-space launch file, so it's not shelled out to). `planning_scene_setup` does **not** exit after populating the scene (`rclcpp::spin()` runs immediately after construction) — neither node-presence nor process-exit is a valid readiness signal here, only the scene's actual contents are. |
| `bringup_aruco_pipeline.launch.py` | `aruco_perception` | `aruco_detector_node` → `calibration_broadcaster_node` | Polls `ros2 node list`-equivalent (`rclpy`'s `get_node_names()`) for `aruco_detector_node` before starting the broadcaster — replicates `wait_for_node.sh aruco_detector_node`. |
| `bringup_yolo_pipeline.launch.py` | `aruco_perception_yolo_bridge` | `inference_server.py` (via `ExecuteProcess`) → `yolo_marker_bridge_node` | `inference_server.py` is **not a ROS node** (plain Flask process inside `~/yolo_venv`, no `rclpy` import — see `start_inference_server.sh`'s header) so it can't use `launch_ros`'s `Node` action at all; started via `ExecuteProcess` invoking the venv's `python3` directly. Readiness = polling `GET http://127.0.0.1:8600/health` for `"status": "ok"` — replicates `wait_for_inference_server.sh`. Then also waits for `move_group` before starting the bridge node, replicating that pane's full `wait_for_inference_server.sh && wait_for_node.sh move_group` chain. |
| `bringup_orchestrator_pipeline.launch.py` | `orchestrator` | includes both pipelines above → `calibration_orchestrator_node` | Cross-package: includes `bringup_moveit_pipeline.launch.py` + `bringup_aruco_pipeline.launch.py`, then polls for **both** `calibration_broadcaster_node` and `trajectory_planner` before starting the orchestrator — replicates `wait_for_node.sh calibration_broadcaster_node && wait_for_node.sh trajectory_planner`. This is the file that transitively brings up the whole moveit+aruco+orchestrator chain from one `ros2 launch` call. |
| `bringup_full_sim.launch.py` / `bringup_full_real.launch.py` | `visual_calibration_moveit` (hosts the top-level files by choice, not because it "owns" the whole pipeline) | includes `bringup_orchestrator_pipeline.launch.py` + `bringup_yolo_pipeline.launch.py` | Top-level sequencer, Python form. Can add real cross-file event-handler gates directly if a per-package file's own internal gate ever isn't sufficient — Python's actual advantage over the XML form below. |
| `bringup_full_sim.xml` / `bringup_full_real.xml` | `visual_calibration_moveit` | same as above | Top-level sequencer, XML form — plain `<include>` tags in the same order, no ordering logic of its own. |

## XML vs. Python — the caveat to verify

`<include>` in XML launch syntax has **no mechanism to wait for an included
launch file to become functionally ready** before starting the next
`<include>` — it only sequences file *inclusion*, all includes are
processed essentially at once, not gated on each other. This is why the
`bringup_full_*.xml` files work at all: every cross-package/cross-node
readiness gate described in the table above already lives **inside** the
relevant `bringup_*.launch.py` file itself (each one blocks internally via
`OpaqueFunction` before emitting its own downstream `Node`/`Include`
actions), so the top-level XML file doesn't need to add any ordering of its
own — it just needs the *files* listed, and each file handles its own
internal sequencing regardless of which top-level form invoked it.

This should hold given the per-file design above, but it's exactly the
thing to verify empirically: run `bringup_full_sim.xml` and
`bringup_full_sim.launch.py` independently against a fresh base session and
compare — if either produces a race the other doesn't, that's the answer to
whether XML's lack of cross-include waiting matters in practice for this
pipeline. If a real difference shows up, prefer the Python form (it can
express additional cross-file ordering XML fundamentally cannot).

## Testing

Recommended order (riskiest gate first): `bringup_moveit_pipeline.launch.py`
→ `bringup_aruco_pipeline.launch.py` → `bringup_yolo_pipeline.launch.py` →
`bringup_orchestrator_pipeline.launch.py` → both top-level files. Each can
be run standalone via `ros2 launch <package> <file> env:=sim` (or `env:=real`)
against an already-running base session, same as testing any other launch
file in this project.
