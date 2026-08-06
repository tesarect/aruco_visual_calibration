[← Back to index](./README.md)

# Architecture

## Project structure

Everything below is under `ros2_ws/src/visual_calibration/`.

```
visual_calibration/
├── aruco_perception/            # Classical detection + TF-chaining pipeline
│   ├── src/aruco_detector/       # ArUco marker detection + pose estimation node
│   ├── src/image_subscriber/     # Minimal camera image/camera_info smoke-test node
│   ├── src/cup_holder_detector/  # Sim-only classical CV cupholder/hole detector
│   ├── src/calibration_broadcaster/  # Chains marker pose with known TF, broadcasts result
│   ├── launch/                   # Per-node launch files (env:=sim|real)
│   └── config/*_sim.yaml, *_real.yaml  # Per-node parameters (real: aruco_detector, calibration_broadcaster only)
├── aruco_perception_yolo_bridge/  # YOLO-backed drop-in alternative marker detector +
│   │                                 cupholder/hole detector (real) + on-demand ~/detect_marker_once
│   └── aruco_perception_yolo_bridge/yolo_marker_bridge_node.py
├── orchestrator/                # Chains cal_ready -> auto-center -> calibrate into one action;
│                                   also the classical/hybrid detector switch, YOLO inference-server
│                                   SIGSTOP/SIGCONT pause, and the rosbridge-reachable web facade
├── depth_perception/             # Back-projects cupholder/hole 2D detections to stable 3D
│                                   positions (rolling-window + hold-last-known filtering),
│                                   broadcasts base_link -> cup_holder -> hole_1..hole_4 TFs
├── visual_calibration_moveit/   # MoveIt2 interaction nodes
│   ├── src/planning_scene_setup/ # Publishes cafeteria collision objects to the planning scene
│   ├── src/trajectory_planner/   # Services to plan/execute moves relative to a TF frame or preset
│   └── src/mtc_trajectory/       # MoveIt Task Constructor node; see visual_calibration_moveit.md
├── visual_calibration_msgs/     # Custom action/srv/msg definitions shared by the above
├── aruco_moveit_config/         # Project's original MoveIt2 config for UR3e + RG2 gripper
├── sim_ur3e_moveit_config/      # Project-owned copy of ur3e_moveit_config, sim-only
├── real_ur3e_moveit_config/     # Project-owned copy of ur3e_moveit_config, real-robot-only
├── calibration_validation/      # Sim-only node: broadcast TF vs. ground-truth TF accuracy check
└── resources/
    ├── docs/                    # This documentation set
    ├── info/                    # Captured TF trees, topic lists, observations (sim vs. real)
    └── scripts/                 # tmux/shell/python helpers for running the sim stack
```

A few packages under `visual_calibration/` are not listed above because their
place in the project isn't settled: `visual_calibration_bringup` (ROS-native
launch sequencing, an alternative to the tmux scripts under
`resources/scripts/tmux/`), `real_moveit_config`, and `real_ur3e_description`
all still exist on disk but were candidates for removal as of the last
review (see `resources/docs/stale_packages_review.md`) — check that file and
confirm current status with the project owner before assuming any of the
three is either gone or a permanent part of the architecture.

Exposing control of the calibration pipeline via a web application is part of
this project's overall goal. That web dashboard (`webpage_ws/`, a separate
npm-managed React app) is developed outside this workspace today and is not
yet a package under `ros2_ws/src/visual_calibration/` — it is expected to be
relocated here later. Until that happens, the control surface for the
pipeline described below is reached the same way any other ROS 2 client
would reach it (CLI, `rosbridge`, etc.), not through any web-app-specific
package, node, or launch file living in this directory.

## Dependency on the wider workspace

- **`ur_description`** (`Universal_Robots_ROS2_Description`) — supplies the
  UR3e xacro (`ur.urdf.xacro`) that `aruco_moveit_config` is generated
  against, matching the RG2-gripper robot actually spawned in Gazebo.
- **`rg2_gripper_description`** — defines `rg2_gripper_aruco_link`, the frame
  the ArUco marker is rigidly mounted at (45 mm marker, 4x4 dictionary —
  50/100/250/1000 depending on config).
- **`the_construct_office_gazebo`** — the Starbots Cafeteria world and the
  launch chain that spawns the simulated UR3e with its wrist-mounted RGBD
  camera; also the source of the cafeteria collision meshes/SDF referenced by
  `planning_scene_setup` (coffee machine, cupholder, countertop, wall).
- **`ur3e_moveit_config`** — the sim/robot-driver-facing MoveIt config;
  `aruco_moveit_config` is this project's own equivalent, kept in sync with
  the same URDF source.
- **`zenoh-pointcloud`** — the real-robot camera bridge. Camera topics on the
  real UR3e cell are only reachable via this Zenoh bridge, not native DDS —
  see [aruco_perception.md](./aruco_perception.md) for how the per-node
  `env:=sim|real` parameter files are organized around that split.
- **YOLO-pipeline** (outside this workspace, an isolated `~/yolo_venv`) —
  runs `inference_server.py`, a plain HTTP inference server
  `aruco_perception_yolo_bridge`'s `yolo_marker_bridge_node` calls into.
  Kept as a separate process/venv rather than a ROS node importing
  `ultralytics` directly, so that ROS's system OpenCV (which `cv_bridge`/
  `image_transport` are compiled against) never shares a process with
  `ultralytics`' bundled, ABI-incompatible newer OpenCV — see
  [aruco_perception_yolo_bridge.md](./aruco_perception_yolo_bridge.md).

## Working / flow

The diagram below covers `orchestrator` sequencing the full auto-calibrate
run, `aruco_perception`/`aruco_perception_yolo_bridge` detecting the marker
and chaining TFs, `trajectory_planner` executing the sampling/centering
moves, and `calibration_validation`'s automated accuracy check.
`calibration_broadcaster_node`'s own internal two-phase sampling loop
(polygon corners, then randomized offsets, with an early-stop check) is
described separately in
[calibration_process.md](./calibration_process.md) and
[aruco_perception.md](./aruco_perception.md) rather than expanded here.

```mermaid
flowchart TD
    subgraph Sim["Gazebo Sim (Starbots Cafeteria)"]
        CAM["/wrist_rgbd_depth_sensor/image_raw + camera_info"]
        TF_KNOWN["/tf: base_link -> ... -> rg2_gripper_aruco_link\n(known from joint states)"]
        TF_GT["/tf: base_link -> wrist_rgbd_camera_depth_optical_frame\n(ground truth, sim only)"]
    end

    subgraph AP["aruco_perception / aruco_perception_yolo_bridge"]
        DET["aruco_detector_node (classical)\nOR yolo_marker_bridge_node (hybrid)\n-- exactly one \"active\" at a time"]
        CB["calibration_broadcaster_node\n(~/calibrate action server)"]
    end

    subgraph ORCH["orchestrator"]
        ORC["calibration_orchestrator_node\n(~/auto_calibrate action +\n~/start_auto_calibrate rosbridge facade)"]
    end

    subgraph MV["visual_calibration_moveit"]
        PSS["planning_scene_setup\n(cafeteria collision objects)"]
        TP["trajectory_planner\n(~/trace_path, ~/get_polygon_waypoints,\n~/get_standoff_pose, ~/move_to_preset)"]
    end

    CAM -->|image, camera_info| DET
    DET -->|"/aruco_perception/marker_pose\n(camera -> marker)"| CB
    DET -->|"/aruco_perception/detections_2d\n(pixel centroid, for image-based centering)"| ORC
    TF_KNOWN -->|"lookupTransform\nbase_link -> rg2_gripper_aruco_link"| CB
    CB -->|"broadcasts static TF\nbase_link -> camera_frame_calibrated\n(samples averaged, early-stop possible)"| TF_OUT["/tf: base_link -> ..._calibrated\n(computed)"]

    ORC -->|"1. move to cal_ready (preset or TF-derived)"| TP
    ORC -->|"2. auto-center on marker (uncalibrated IBVS,\nimage Jacobian from 2 bootstrap probes)"| TP
    ORC -->|"3. ~/calibrate goal, relays feedback/result"| CB
    ORC -.->|"~/set_detector_mode\n(flips \"active\" param on both detectors)"| DET

    CB -->|"~/get_polygon_waypoints (read-only),\nthen ~/trace_path per waypoint\n(blocks until settled)"| TP
    TP -.->|"lookupTransform camera_frame\nin planning frame"| TF_OUT
    TP -->|MoveGroupInterface plan+execute| ARM["UR3e arm motion"]
    PSS -->|collision objects| ARM

    subgraph CV["calibration_validation"]
        VALIDATE["validate_calibration_sim.py\n(one-shot position + orientation\nerror vs. ground truth)"]
    end
    TF_OUT --> VALIDATE
    TF_GT --> VALIDATE

    ARM -->|"settled pose triggers\na fresh marker detection"| DET
```

Flow narrative:

1. Exactly one of the two marker detectors is "active" at a time —
   `aruco_detector_node` (classical OpenCV ArUco) or
   `yolo_marker_bridge_node` (calls a YOLO model over HTTP) — both publish
   the same `geometry_msgs/PoseStamped` shape on
   `/aruco_perception/marker_pose`, so `calibration_broadcaster_node` needs
   no changes regardless of which one produced it.
   `calibration_orchestrator_node`'s `~/set_detector_mode` flips the switch
   by setting one node's `active` parameter true and the other's false —
   see [aruco_perception_yolo_bridge.md](./aruco_perception_yolo_bridge.md).
2. A caller normally starts the whole sequence via
   `calibration_orchestrator_node`'s `~/auto_calibrate` action (or its
   `~/start_auto_calibrate` rosbridge-reachable facade, for clients like the
   web app that can't speak rosbridge's native ROS2 action protocol — see
   [orchestrator.md](./orchestrator.md)): move to `cal_ready`, optionally
   auto-center on the marker using an uncalibrated image-based visual
   servoing search, then call `calibration_broadcaster_node`'s
   `~/calibrate` action and relay its feedback/result.
3. `calibration_broadcaster_node` orchestrates that `~/calibrate` action
   goal: it fetches waypoints from `trajectory_planner`'s
   `~/get_polygon_waypoints` (read-only, no motion), then runs a polygon
   phase followed by a randomized-offset phase, checking an early-stop
   condition after every sample — see
   [calibration_process.md](./calibration_process.md) for the full
   per-sample mechanism (still: trace to a waypoint, wait for a *fresh*
   marker detection, chain it with the known TF chain).
4. Once enough samples are collected (or early-stop triggers), position is
   averaged arithmetically and orientation is averaged by the configured
   quaternion-averaging method, and a static TF
   `known_chain_frame → camera` is broadcast.
5. In simulation, `calibration_validation`'s `validate_calibration_sim.py`
   node automatically compares the computed `base_link → camera_..._calibrated`
   TF against Gazebo's ground-truth `base_link → wrist_rgbd_camera_depth_optical_frame`
   TF, logging a position error (cm) and orientation error (deg) with a
   GOOD/CHECK/BAD verdict — see
   [calibration_validation.md](./calibration_validation.md).
6. `planning_scene_setup` runs independently to keep MoveIt aware of cafeteria
   obstacles (coffee machine, cupholder, countertop, wall — plus, on real,
   an unmeasured placeholder box guarding the wall-mounted camera) during
   any of the above arm motion.

A separate, parallel pipeline (not shown above) uses that same computed
`camera → base_link` TF: the active detector's `cup_holder`/`hole` 2D pixel
detections (from sim's `cup_holder_detector_node` or real's
`yolo_marker_bridge_node`) feed `depth_perception_node`, which back-projects
them to 3D, filters them over time, and chains them through the calibrated
camera TF to broadcast `base_link → cup_holder → hole_1..hole_4` — see
[depth_perception.md](./depth_perception.md).
