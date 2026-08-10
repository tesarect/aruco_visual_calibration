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
│   └── src/mtc_trajectory/       # MoveIt Task Constructor node; not currently built, see visual_calibration_moveit.md
├── visual_calibration_msgs/     # Custom action/srv/msg definitions shared by the above
├── sim_ur3e_moveit_config/      # Project-owned copy of ur3e_moveit_config, sim-only
├── real_ur3e_moveit_config/     # Project-owned copy of ur3e_moveit_config, real-robot-only
├── calibration_validation/      # Sim-only node: broadcast TF vs. ground-truth TF accuracy check
└── resources/
    ├── docs/                    # This documentation set
    ├── info/                    # Captured TF trees, topic lists, observations (sim vs. real)
    └── scripts/                 # tmux/shell/python helpers for running the sim stack
```

`sim_ur3e_moveit_config` / `real_ur3e_moveit_config` are what `move_group`
actually launches from — see
[ur3e_moveit_config_variants.md](./ur3e_moveit_config_variants.md).

A few packages under `visual_calibration/` are not listed above because their
place in the project isn't settled: `visual_calibration_bringup` (ROS-native
launch sequencing, an alternative to the tmux scripts under
`resources/scripts/tmux/` — see
[manual_bringup.md](./manual_bringup.md)), and `real_ur3e_description` still
exist on disk but were candidates for removal as of the last review (see
`resources/docs/stale_packages_review.md`) — check that file and confirm
current status with the project owner before assuming either is either gone
or a permanent part of the architecture.

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
  UR3e xacro (`ur.urdf.xacro`) that `sim_ur3e_moveit_config`/
  `real_ur3e_moveit_config` trace back to, matching the RG2-gripper robot
  actually spawned in Gazebo.
- **`rg2_gripper_description`** — defines `rg2_gripper_aruco_link`, the frame
  the ArUco marker is rigidly mounted at (45 mm marker, 4x4 dictionary —
  50/100/250/1000 depending on config).
- **`the_construct_office_gazebo`** — the Starbots Cafeteria world and the
  launch chain that spawns the simulated UR3e with its wrist-mounted RGBD
  camera; also the source of the cafeteria collision meshes/SDF referenced by
  `planning_scene_setup` (coffee machine, cupholder, countertop, wall).
- **`ur3e_moveit_config`** — the instructor-provided, robot-driver-facing
  MoveIt config; `sim_ur3e_moveit_config`/`real_ur3e_moveit_config` are this
  project's own environment-split copies, kept in sync with the same URDF
  source — see
  [ur3e_moveit_config_variants.md](./ur3e_moveit_config_variants.md).
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

## Component interaction, at a glance

The diagram below is deliberately low-detail: boxes are packages/nodes,
arrows are just "who talks to whom," with no topic/service/action names.
See the detailed diagram further down for the actual interfaces.

```mermaid
flowchart LR
    subgraph AP["aruco_perception"]
        DETC["aruco_detector_node"]
        CUPH["cup_holder_detector"]
        CB["calibration_broadcaster_node"]
        IMGSUB["image_subscriber_node"]
    end

    subgraph YOLO["aruco_perception_yolo_bridge"]
        BRIDGE["yolo_marker_bridge_node"]
    end

    subgraph ORCH["orchestrator"]
        ORC["calibration_orchestrator_node"]
    end

    subgraph MV["visual_calibration_moveit"]
        PSS["planning_scene_setup"]
        TP["trajectory_planner"]
        MTC["mtc_trajectory"]
    end

    subgraph DP["depth_perception"]
        DPN["depth_perception_node"]
    end

    subgraph CV["calibration_validation"]
        VAL["validate_calibration_sim.py"]
    end

    Camera(["Camera feed"]) --> DETC
    Camera --> CUPH
    Camera --> BRIDGE
    Camera --> IMGSUB

    DETC --> CB
    BRIDGE --> CB
    ORC --> CB
    ORC --> TP
    ORC --> DETC
    ORC --> BRIDGE
    CB --> TP

    DETC --> DPN
    CUPH --> DPN
    BRIDGE --> DPN
    DPN --> TP

    PSS --- TP
    MTC -.-> TP

    CB --> VAL
    TP --> Arm(["UR3e arm"])
```

`mtc_trajectory` is drawn with a dashed arrow because it is not currently
built (see [visual_calibration_moveit.md](./visual_calibration_moveit.md));
it is included here as a real package/node on disk, not as an active part
of the runtime data flow.

## Detailed architecture

The diagram below covers `orchestrator` sequencing the full auto-calibrate
run, `aruco_perception`/`aruco_perception_yolo_bridge` detecting the marker
and chaining TFs, `trajectory_planner` executing the sampling/centering
moves, `depth_perception` chaining the calibrated camera TF into 3D
cupholder/hole positions, and `calibration_validation`'s automated accuracy
check. `calibration_broadcaster_node`'s own internal two-phase sampling loop
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

    subgraph AP["aruco_perception"]
        DET["aruco_detector_node (classical)"]
        CUPH["cup_holder_detector\n(sim only)"]
        CB["calibration_broadcaster_node\n(~/calibrate action server)"]
    end

    subgraph YOLO["aruco_perception_yolo_bridge"]
        BRIDGE["yolo_marker_bridge_node (hybrid)\n-- also cupholder/hole on real,\n~/detect_marker_once"]
    end

    subgraph ORCH["orchestrator"]
        ORC["calibration_orchestrator_node\n(~/auto_calibrate action +\n~/start_auto_calibrate rosbridge facade)"]
    end

    subgraph MV["visual_calibration_moveit"]
        PSS["planning_scene_setup\n(cafeteria collision objects)"]
        TP["trajectory_planner\n(~/trace_path, ~/get_polygon_waypoints,\n~/get_standoff_pose, ~/get_preset_pose,\n~/move_to_preset, ~/move_to_instance)"]
    end

    subgraph DP["depth_perception"]
        DPN["depth_perception_node"]
    end

    subgraph CV["calibration_validation"]
        VALIDATE["validate_calibration_sim.py\n(one-shot position + orientation\nerror vs. ground truth)"]
    end

    CAM -->|image, camera_info| DET
    CAM -->|image, camera_info| CUPH
    CAM -->|image, camera_info| BRIDGE

    DET -->|"/aruco_perception/marker_pose\n(camera -> marker, classical mode)"| CB
    BRIDGE -->|"/aruco_perception/marker_pose\n(camera -> marker, hybrid mode)"| CB
    DET -.->|"exactly one &quot;active&quot; at a time\n(active param)"| BRIDGE

    DET -->|"/aruco_perception/detections_2d\n(marker pixel centroid)"| ORC
    TF_KNOWN -->|"lookupTransform\nbase_link -> rg2_gripper_aruco_link"| CB
    CB -->|"broadcasts static TF\nbase_link -> camera_frame_calibrated\n(samples averaged, early-stop possible)"| TF_OUT["/tf: base_link -> ..._calibrated\n(computed)"]

    ORC -->|"1. ~/get_standoff_pose,\n~/trace_path or ~/move_to_preset\n(move to cal_ready)"| TP
    ORC -->|"2. auto-center on marker (uncalibrated IBVS,\nimage Jacobian from bootstrap probes)"| TP
    ORC -->|"3. ~/calibrate goal, relays feedback/result"| CB
    ORC -.->|"~/set_detector_mode\n(flips &quot;active&quot; param on both detectors)"| DET
    ORC -.->|"~/set_detector_mode"| BRIDGE

    CB -->|"~/get_polygon_waypoints (read-only),\nthen ~/trace_path per waypoint\n(blocks until settled)"| TP
    CB -.->|"~/detect_marker_once\n(hybrid_per_waypoint_enabled mode only)"| BRIDGE
    TP -.->|"lookupTransform camera_frame\nin planning frame"| TF_OUT
    TP -->|MoveGroupInterface plan+execute| ARM["UR3e arm motion"]
    PSS -->|collision objects| ARM

    TF_OUT --> VALIDATE
    TF_GT --> VALIDATE

    DET -->|"detections_2d\n(cup_holder/hole, real)"| DPN
    CUPH -->|"detections_2d\n(cup_holder/hole, sim)"| DPN
    BRIDGE -->|"detections_2d\n(cup_holder/hole, real)"| DPN
    TF_OUT -.->|"calibrated camera TF\n(chains 3D back-projection)"| DPN
    DPN -->|"/tf: base_link -> cup_holder -> hole_1..hole_4"| TF_HOLES["/tf: cup_holder / hole frames"]

    ARM -->|"settled pose triggers\na fresh marker detection"| DET
    ARM -->|"settled pose triggers\na fresh marker detection"| BRIDGE
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
   [orchestrator.md](./orchestrator.md)): move to `cal_ready` (via
   `~/get_standoff_pose` + `~/trace_path`, or `~/move_to_preset`), optionally
   auto-center on the marker using an uncalibrated image-based visual
   servoing search, then call `calibration_broadcaster_node`'s
   `~/calibrate` action and relay its feedback/result — see
   [calibration_process.md](./calibration_process.md) for that hand-off in
   sequence-diagram form.
3. `calibration_broadcaster_node` orchestrates that `~/calibrate` action
   goal: it fetches waypoints from `trajectory_planner`'s
   `~/get_polygon_waypoints` (read-only, no motion), then runs a polygon
   phase followed by a randomized-offset phase, checking an early-stop
   condition after every sample — see
   [calibration_process.md](./calibration_process.md) for the full
   per-sample mechanism (still: trace to a waypoint, wait for a *fresh*
   marker detection, chain it with the known TF chain). In hybrid mode with
   `hybrid_per_waypoint_enabled`, it can additionally call
   `yolo_marker_bridge_node`'s `~/detect_marker_once` directly per waypoint.
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
7. A separate, parallel pipeline uses that same computed `camera → base_link`
   TF: the active detector's `cup_holder`/`hole` 2D pixel detections (from
   sim's `cup_holder_detector_node` or, on both sim and real,
   `yolo_marker_bridge_node`) feed `depth_perception_node`, which
   back-projects them to 3D, filters them over time, and chains them through
   the calibrated camera TF to broadcast `base_link → cup_holder →
   hole_1..hole_4` — see [depth_perception.md](./depth_perception.md).

## Pipeline stages

The diagram below is the "what happens, in order" view of one `~/calibrate`
run — one box per processing stage rather than per node/package. See
[calibration_process.md](./calibration_process.md) for the full
plain-language walkthrough of each stage.

```mermaid
flowchart TD
    A["Camera image in\n(RGB frame + camera_info)"]
    B["Marker detection\n(classical ArUco or YOLO,\nexactly one active)\n-> camera -> marker pose"]
    C["TF chaining\n(known base_link -> marker chain,\nfrom joint states, combined with\nthe fresh detection)\n-> one base_link -> camera sample"]
    D["Sample collection\n(move to next waypoint,\nwait for a settled + fresh detection,\nrepeat until num_samples)"]
    E["Averaging\n(position: arithmetic mean;\norientation: quaternion averaging,\ndouble-cover corrected)"]
    F["Broadcast TF out\n(static base_link -> camera_..._calibrated)"]

    A --> B --> C --> D --> E --> F
```
