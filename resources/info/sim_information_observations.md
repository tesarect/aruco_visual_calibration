 ros2 control list_controllers
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
joint_trajectory_controller[joint_trajectory_controller/JointTrajectoryController] active
gripper_controller  [position_controllers/GripperActionController] active

ros2 topic echo /joint_states --once
A message was lost!!!
        total count change:1
        total count: 1---
header:
  stamp:
    sec: 34
    nanosec: 866000000
  frame_id: ''
name:
- shoulder_pan_joint
- shoulder_lift_joint
- elbow_joint
- wrist_1_joint
- wrist_2_joint
- wrist_3_joint
- rg2_gripper_finger_left_joint
position:
- 3.701613991413666e-05
- -1.5699998667240649
- -6.766350706399749e-07
- -1.569999723037645
- -4.258066961693174e-05
- -4.58721656162453e-05
- -0.00013109844878034238
velocity:
- 0.00017654151675300438
- 0.00013327593737566637
- -0.0002075459984444127
- 0.00027696235508729866
- -0.00017675272892632215
- -0.00021267293201151306
- 1.8774447088115763e-05
effort:
- .nan
- .nan
- .nan
- .nan
- .nan
- .nan
- .nan
---

ros2 topic list
/clock
/demo/link_states_demo
/demo/model_states_demo
/dynamic_joint_states
/gripper_controller/transition_event
/joint_state_broadcaster/transition_event
/joint_states
/joint_trajectory_controller/controller_state
/joint_trajectory_controller/joint_trajectory
/joint_trajectory_controller/state
/joint_trajectory_controller/transition_event
/parameter_events
/performance_metrics
/robot_description
/rosout
/tf
/tf_static
/wrist_rgbd_depth_sensor/camera_info
/wrist_rgbd_depth_sensor/depth/camera_info
/wrist_rgbd_depth_sensor/depth/image_raw
/wrist_rgbd_depth_sensor/depth/image_raw/compressed
/wrist_rgbd_depth_sensor/depth/image_raw/compressedDepth
/wrist_rgbd_depth_sensor/depth/image_raw/theora
/wrist_rgbd_depth_sensor/image_raw
/wrist_rgbd_depth_sensor/image_raw/compressed
/wrist_rgbd_depth_sensor/image_raw/compressedDepth
/wrist_rgbd_depth_sensor/image_raw/theora
/wrist_rgbd_depth_sensor/points

tf2 frames:
digraph G {
"upper_arm_link" -> "forearm_link"[label=" Broadcaster: default_authority\nAverage rate: 57.085\nBuffer length: 2.47\nMost recent transform: 538.916\nOldest transform: 536.446\n"];
"shoulder_link" -> "upper_arm_link"[label=" Broadcaster: default_authority\nAverage rate: 57.085\nBuffer length: 2.47\nMost recent transform: 538.916\nOldest transform: 536.446\n"];
"rg2_gripper_base_link" -> "rg2_gripper_left_finger"[label=" Broadcaster: default_authority\nAverage rate: 57.085\nBuffer length: 2.47\nMost recent transform: 538.916\nOldest transform: 536.446\n"];
"tool0" -> "rg2_gripper_base_link"[label=" Broadcaster: default_authority\nAverage rate: 10000.0\nBuffer length: 0.0\nMost recent transform: 0.0\nOldest transform: 0.0\n"];
"rg2_gripper_base_link" -> "rg2_gripper_right_finger"[label=" Broadcaster: default_authority\nAverage rate: 57.085\nBuffer length: 2.47\nMost recent transform: 538.916\nOldest transform: 536.446\n"];
"rg2_gripper_left_finger" -> "rg2_gripper_left_thumb"[label=" Broadcaster: default_authority\nAverage rate: 57.085\nBuffer length: 2.47\nMost recent transform: 538.916\nOldest transform: 536.446\n"];
"rg2_gripper_right_finger" -> "rg2_gripper_right_thumb"[label=" Broadcaster: default_authority\nAverage rate: 57.085\nBuffer length: 2.47\nMost recent transform: 538.916\nOldest transform: 536.446\n"];
"base_link_inertia" -> "shoulder_link"[label=" Broadcaster: default_authority\nAverage rate: 57.085\nBuffer length: 2.47\nMost recent transform: 538.916\nOldest transform: 536.446\n"];
"base_link" -> "base_link_inertia"[label=" Broadcaster: default_authority\nAverage rate: 10000.0\nBuffer length: 0.0\nMost recent transform: 0.0\nOldest transform: 0.0\n"];
"forearm_link" -> "wrist_1_link"[label=" Broadcaster: default_authority\nAverage rate: 57.085\nBuffer length: 2.47\nMost recent transform: 538.916\nOldest transform: 536.446\n"];
"wrist_1_link" -> "wrist_2_link"[label=" Broadcaster: default_authority\nAverage rate: 57.085\nBuffer length: 2.47\nMost recent transform: 538.916\nOldest transform: 536.446\n"];
"wrist_2_link" -> "wrist_3_link"[label=" Broadcaster: default_authority\nAverage rate: 57.085\nBuffer length: 2.47\nMost recent transform: 538.916\nOldest transform: 536.446\n"];
"world" -> "base_link"[label=" Broadcaster: default_authority\nAverage rate: 10000.0\nBuffer length: 0.0\nMost recent transform: 0.0\nOldest transform: 0.0\n"];
"base_link" -> "base"[label=" Broadcaster: default_authority\nAverage rate: 10000.0\nBuffer length: 0.0\nMost recent transform: 0.0\nOldest transform: 0.0\n"];
"flange" -> "tool0"[label=" Broadcaster: default_authority\nAverage rate: 10000.0\nBuffer length: 0.0\nMost recent transform: 0.0\nOldest transform: 0.0\n"];
"wrist_3_link" -> "flange"[label=" Broadcaster: default_authority\nAverage rate: 10000.0\nBuffer length: 0.0\nMost recent transform: 0.0\nOldest transform: 0.0\n"];
"rg2_gripper_base_link" -> "rg2_gripper_aruco_link"[label=" Broadcaster: default_authority\nAverage rate: 10000.0\nBuffer length: 0.0\nMost recent transform: 0.0\nOldest transform: 0.0\n"];
"wrist_3_link" -> "ft_frame"[label=" Broadcaster: default_authority\nAverage rate: 10000.0\nBuffer length: 0.0\nMost recent transform: 0.0\nOldest transform: 0.0\n"];
"wrist_rgbd_camera_link" -> "wrist_rgbd_camera_depth_frame"[label=" Broadcaster: default_authority\nAverage rate: 10000.0\nBuffer length: 0.0\nMost recent transform: 0.0\nOldest transform: 0.0\n"];
"base_link" -> "wrist_rgbd_camera_link"[label=" Broadcaster: default_authority\nAverage rate: 10000.0\nBuffer length: 0.0\nMost recent transform: 0.0\nOldest transform: 0.0\n"];
"wrist_rgbd_camera_depth_frame" -> "wrist_rgbd_camera_depth_optical_frame"[label=" Broadcaster: default_authority\nAverage rate: 10000.0\nBuffer length: 0.0\nMost recent transform: 0.0\nOldest transform: 0.0\n"];
"wrist_rgbd_camera_depth_frame" -> "wrist_rgbd_camera_fisheye_frame"[label=" Broadcaster: default_authority\nAverage rate: 10000.0\nBuffer length: 0.0\nMost recent transform: 0.0\nOldest transform: 0.0\n"];
"wrist_rgbd_camera_fisheye_frame" -> "wrist_rgbd_camera_fisheye_optical_frame"[label=" Broadcaster: default_authority\nAverage rate: 10000.0\nBuffer length: 0.0\nMost recent transform: 0.0\nOldest transform: 0.0\n"];
"wrist_rgbd_camera_depth_frame" -> "wrist_rgbd_camera_left_ir_frame"[label=" Broadcaster: default_authority\nAverage rate: 10000.0\nBuffer length: 0.0\nMost recent transform: 0.0\nOldest transform: 0.0\n"];
"wrist_rgbd_camera_left_ir_frame" -> "wrist_rgbd_camera_left_ir_optical_frame"[label=" Broadcaster: default_authority\nAverage rate: 10000.0\nBuffer length: 0.0\nMost recent transform: 0.0\nOldest transform: 0.0\n"];
"wrist_rgbd_camera_depth_frame" -> "wrist_rgbd_camera_right_ir_frame"[label=" Broadcaster: default_authority\nAverage rate: 10000.0\nBuffer length: 0.0\nMost recent transform: 0.0\nOldest transform: 0.0\n"];
"wrist_rgbd_camera_right_ir_frame" -> "wrist_rgbd_camera_right_ir_optical_frame"[label=" Broadcaster: default_authority\nAverage rate: 10000.0\nBuffer length: 0.0\nMost recent transform: 0.0\nOldest transform: 0.0\n"];
edge [style=invis];
 subgraph cluster_legend { style=bold; color=black; label ="view_frames Result";
"Recorded at time: 1782832214.361522"[ shape=plaintext ] ;
}->"world";
}