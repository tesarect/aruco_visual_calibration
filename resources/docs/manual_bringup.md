[← Back to index](./README.md)

# Manual bringup

Two ways exist to bring the calibration stack up: a manual, ordered sequence
of individual `ros2 launch`/`ros2 run` commands (below), or the
launch-native `visual_calibration_bringup` package, which sequences the same
nodes from launch files instead of separate shell steps. This page covers
the manual sequence; `visual_calibration_bringup`'s own internals aren't
covered here.

Each step below assumes the previous ones are already up and settled
(Gazebo/`move_group` in particular can take a while — give them a few
seconds before starting the next step). Aliases (see
`resources/scripts/shell/aliases.sh`) are shown alongside the raw commands.
For a scripted/tmux-based startup that gates each step on the right
readiness signal instead of guessing, see
`resources/scripts/tmux/sim_tmux_base.sh` + `sim_tmux_trajcal.sh` (aliases:
`tmuxbasesim`, `tmuxtrajcalsim`).

1. **Simulation** — `startsim` / `ros2 launch the_construct_office_gazebo starbots_ur3e.launch.xml`
2. **MoveIt2 move_group** — `startmoveitgroup` / `ros2 launch sim_ur3e_moveit_config move_group.launch.py`
   (use `real_ur3e_moveit_config` on the real robot)
3. **RViz** — `startrviz` / `ros2 launch sim_ur3e_moveit_config moveit_rviz.launch.py`
4. **Marker debugger (optional)** — `python3 resources/scripts/python/tf_debug_markers.py`
5. **Planning scene** — `startplanningscene` / `ros2 launch visual_calibration_moveit planning_scene_setup.launch.py env:=sim`
6. **Trajectory planner** — `starttrajectoryplanner` / `ros2 launch visual_calibration_moveit trajectory_planner.launch.py env:=sim`
   (this alone doesn't move the arm — see step 9)
7. **ArUco detector** — `startarucodetector` / `ros2 run aruco_perception aruco_detector_node --ros-args --params-file .../config/aruco_detector_sim.yaml`
   — detects the marker, publishes `/aruco_perception/marker_pose`; add `-p publish_overlay_image:=true` (or the `viewoverlaycam` alias to view it) for the debug overlay
8. **Calibration broadcaster** — `startcalibrationbroadcaster` / `ros2 run aruco_perception calibration_broadcaster_node --ros-args --params-file .../config/calibration_broadcaster_sim.yaml`
9. **Calibration orchestrator (optional, recommended)** — `startcalibrationorchestrator` /
   `ros2 run orchestrator calibration_orchestrator_node --ros-args --params-file .../config/calibration_orchestrator_sim.yaml`
   — chains "move to cal_ready" → optional auto-center → `~/calibrate` into one
   `~/auto_calibrate` action, see [orchestrator.md](./orchestrator.md).
10. **Start calibration**:
    - Via the orchestrator (from step 9): `startautocalibration` (blocks,
      `ros2 action send_goal /calibration_orchestrator_node/auto_calibrate
      visual_calibration_msgs/action/AutoCalibrate {} --feedback`) — moves to
      `cal_ready` itself, no manual positioning needed first.
    - Directly against `calibration_broadcaster_node` (skips steps 9 and the
      cal_ready move — the arm must already be positioned so the marker is
      visible): `startcalibration` (blocks, `ros2 action send_goal
      /calibration_broadcaster_node/calibrate
      visual_calibration_msgs/action/Calibrate {} --feedback`).

    Either way, `calibration_broadcaster_node` runs the sampling loop itself:
    it fetches `trajectory_planner`'s polygon waypoints
    (`~/get_polygon_waypoints`, read-only), then for each one calls
    `~/trace_path` with just that pose (blocking until the arm settles there
    — the `~/trace_path` response only arrives once the move actually
    completes, which is what guarantees a sample is never taken mid-motion),
    waits for a fresh `marker_pose` published after that point, and records
    one sample — see [calibration_process.md](./calibration_process.md).

    Prints live `samples_collected/samples_total` feedback, then the final
    result (success, message, `max_spread_deg`/`mean_spread_deg`).
    `~/calibrate` is an action (not a plain service) specifically so a web UI
    can show live progress, not just a final result — see
    `visual_calibration_msgs/action/Calibrate.action`.
