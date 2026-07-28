# Simulation:
1. `startsim` :
```
source ~/ros2_ws/install/setup.bash && ros2 launch the_construct_office_gazebo starbots_ur3e.launch.xml
```

2. `startmoveitconfig` :
```
source ~/ros2_ws/install/setup.bash && ros2 launch sim_ur3e_moveit_config move_group.launch.py
```

3. `startrviz` :
```
source ~/ros2_ws/install/setup.bash && ros2 launch sim_ur3e_moveit_config moveit_rviz.launch.py
```

4. `startplanningscene` :
```
source ~/ros2_ws/install/setup.bash && ros2 launch visual_calibration_moveit planning_scene_setup.launch.py env:=sim
```

5. `starttrajectoryplanner` :
```
source ~/ros2_ws/install/setup.bash && ros2 launch visual_calibration_moveit trajectory_planner.launch.py env:=sim
```

6. `startarucodetector` :
```
source ~/ros2_ws/install/setup.bash && ros2 run aruco_perception aruco_detector_node --ros-args --params-file ~/ros2_ws/src/visual_calibration/aruco_perception/config/aruco_detector_sim.yaml
```

7. (no alias — use `ros2 run` directly, unlike `startcalibrationorchestrator` below this uses `ros2 launch`) `calibration_broadcaster_node` :
```
source ~/ros2_ws/install/setup.bash && ros2 run aruco_perception calibration_broadcaster_node --ros-args --params-file ~/ros2_ws/src/visual_calibration/aruco_perception/config/calibration_broadcaster_sim.yaml
```

8. `calibration_orchestrator.launch.py` (NOTE: `startcalibrationorchestrator` alias uses `ros2 run` + a raw params-file instead — the tmux sessions use this launch-file form; use whichever, they start the same node) :
```
source ~/ros2_ws/install/setup.bash && ros2 launch orchestrator calibration_orchestrator.launch.py env:=sim
```

9. `startinferenceserver` (NOT a ROS node — plain Flask process inside `~/yolo_venv`, do not run from a shell that also sources ROS's setup.bash, see the ABI-isolation note on the alias itself) :
```
bash ~/ros2_ws/src/visual_calibration/aruco_perception_yolo_bridge/resources/scripts/shell/start_inference_server.sh sim
```

10. `startyolomarkerbridge` (requires step 9 already running) :
```
source ~/ros2_ws/install/setup.bash && ros2 run aruco_perception_yolo_bridge yolo_marker_bridge_node.py --ros-args --params-file ~/ros2_ws/src/visual_calibration/aruco_perception_yolo_bridge/config/yolo_marker_bridge_sim.yaml
```

11. `startrosbridge` :
```
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

12. (no alias) web dashboard :
```
cd ~/webpage_ws && bash ./setup_rosject.sh --env sim && source ~/webpage_ws/scripts/session_init.sh && cd ~/webpage_ws/app && npm run start
```

13. (no alias — debug/visual-aid tools, all optional) marker debugger :
```
python3 ~/ros2_ws/src/visual_calibration/resources/scripts/python/tf_debug_markers.py --env sim
```

14. `viewoverlaycam` :
```
ros2 run rqt_image_view rqt_image_view /aruco_perception/overlay_image
```

15. (no alias) rqt graph :
```
ros2 run rqt_graph rqt_graph
```

16. `startswriconsole` :
```
ros2 run swri_console swri_console
```

### Ordering notes (sim)
- **1** (Gazebo) must be up first — **2** (move_group) waits on the sim controller stack coming from it.
- **3** (RViz) and **4** (Planning Scene) both just need **2** — parallel-safe with each other.
- **5** (Trajectory Planner) needs **2** + **4** actually populated (its startup home-move plans against the scene immediately).
- **6** (Aruco Detector) only needs **2** — parallel-safe with **5**.
- **7** (Calibration Broadcaster) needs **5** + **6**.
- **8** (Calibration Orchestrator) needs **5** + **7** — last in the trajcal chain.
- **9** (Inference Server) has no ROS dependency, can start any time; **10** (Yolo Marker Bridge) needs **9** + **2**.
- **11** (ROS Bridge) and **12** (web dashboard) are independent of everything else — start any time, parallel-safe with each other.
- **13**–**16** (debug/visual-aid tools) all just need **2** at minimum to be useful; none block anything else and can be skipped entirely.

# Real :
1. Zenoh bridge (no alias — cd into its own directory first) :
```
cd ~/ros2_ws/src/zenoh-pointcloud/init && ./rosject.sh
```

2. `startmoveitgroup real` (NOTE: on real hardware the tmux session also runs `ensure_controller_active.sh /controller_manager scaled_joint_trajectory_controller` right before this — `scaled_joint_trajectory_controller` has been observed dropping to inactive intermittently on real; if move_group's first motion command fails, run that script manually before retrying) :
```
source ~/ros2_ws/install/setup.bash && ros2 launch real_ur3e_moveit_config move_group.launch.py
```

3. `startrviz real` :
```
source ~/ros2_ws/install/setup.bash && ros2 launch real_ur3e_moveit_config moveit_rviz.launch.py
```

4. `startplanningscene real` :
```
source ~/ros2_ws/install/setup.bash && ros2 launch visual_calibration_moveit planning_scene_setup.launch.py env:=real
```

5. `starttrajectoryplanner real` :
```
source ~/ros2_ws/install/setup.bash && ros2 launch visual_calibration_moveit trajectory_planner.launch.py env:=real
```

6. `startarucodetector real` :
```
source ~/ros2_ws/install/setup.bash && ros2 run aruco_perception aruco_detector_node --ros-args --params-file ~/ros2_ws/src/visual_calibration/aruco_perception/config/aruco_detector_real.yaml
```

7. (no alias) `calibration_broadcaster_node` :
```
source ~/ros2_ws/install/setup.bash && ros2 run aruco_perception calibration_broadcaster_node --ros-args --params-file ~/ros2_ws/src/visual_calibration/aruco_perception/config/calibration_broadcaster_real.yaml
```

8. `calibration_orchestrator.launch.py` (see sim step 8's note — same launch-file-vs-alias discrepancy) :
```
source ~/ros2_ws/install/setup.bash && ros2 launch orchestrator calibration_orchestrator.launch.py env:=real
```

9. `startinferenceserver real` (NOT a ROS node — see sim step 9's ABI-isolation note) :
```
bash ~/ros2_ws/src/visual_calibration/aruco_perception_yolo_bridge/resources/scripts/shell/start_inference_server.sh real
```

10. `startyolomarkerbridge real` (requires step 9 already running) :
```
source ~/ros2_ws/install/setup.bash && ros2 run aruco_perception_yolo_bridge yolo_marker_bridge_node.py --ros-args --params-file ~/ros2_ws/src/visual_calibration/aruco_perception_yolo_bridge/config/yolo_marker_bridge_real.yaml
```

11. `startrosbridge` :
```
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

12. (no alias) web dashboard (NOTE: `real_tmux_webstack.sh`'s pane has no `source ~/ros2_ws/install/setup.bash` prefix and uses `PORT=7000 npm run build && PORT=7000 npm run preview` instead of `npm run start`) :
```
source ~/webpage_ws/scripts/session_init.sh && cd ~/webpage_ws/app && PORT=7000 npm run build && PORT=7000 npm run preview
```

13. (no alias — debug/visual-aid tools, all optional) marker debugger :
```
python3 ~/ros2_ws/src/visual_calibration/resources/scripts/python/tf_debug_markers.py --env real
```

14. `viewoverlaycam` :
```
ros2 run rqt_image_view rqt_image_view /aruco_perception/overlay_image
```

15. (no alias) rqt graph :
```
ros2 run rqt_graph rqt_graph
```

16. `startswriconsole` :
```
ros2 run swri_console swri_console
```

### Ordering notes (real)
- **1** (Zenoh bridge) has no polling dependency on the others in-file, but logically first — downstream `/D415/*` camera topics depend on it, and it assumes the real robot driver is already up externally (see `realrobotstatuscheck`/`check_real_driver.sh` — not included here since this list is direct launch/run commands only, no wait/check scripts).
- **2** (move_group) does not itself poll for anything — start after **1**.
- **3** (RViz) and **4** (Planning Scene) both just need **2** — parallel-safe with each other.
- **5** (Trajectory Planner) needs **2** + **4** actually populated.
- **6** (Aruco Detector) needs **1** (camera topics) — parallel-safe with **5**.
- **7** (Calibration Broadcaster) needs **5** + **6**.
- **8** (Calibration Orchestrator) needs **5** + **7** — last in the trajcal chain.
- **9** (Inference Server) has no ROS dependency, can start any time; **10** (Yolo Marker Bridge) needs **9** + **2**.
- **11** (ROS Bridge) and **12** (web dashboard) are independent of everything else.
- **13**–**16** (debug/visual-aid tools) all just need **2** at minimum to be useful; none block anything else and can be skipped entirely.
