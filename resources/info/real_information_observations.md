ros2 control list_controllers
[TBD]

ros2 topic echo /joint_states --once
[TBD]

ros2 topic list
[TBD]

tf2 frames:
digraph G {
"world" -> "base_link"[label=" Broadcaster: default_authority\nAverage rate: 10000.0\nBuffer length: 0.0\nMost recent transform: 0.0\nOldest transform: 0.0\n"];
"base_link" -> "base"[label=" Broadcaster: default_authority\nAverage rate: 10000.0\nBuffer length: 0.0\nMost recent transform: 0.0\nOldest transform: 0.0\n"];
"base_link" -> "base_link_inertia"[label=" Broadcaster: default_authority\nAverage rate: 10000.0\nBuffer length: 0.0\nMost recent transform: 0.0\nOldest transform: 0.0\n"];
"flange" -> "tool0"[label=" Broadcaster: default_authority\nAverage rate: 10000.0\nBuffer length: 0.0\nMost recent transform: 0.0\nOldest transform: 0.0\n"];
"wrist_3_link" -> "flange"[label=" Broadcaster: default_authority\nAverage rate: 10000.0\nBuffer length: 0.0\nMost recent transform: 0.0\nOldest transform: 0.0\n"];
"tool0" -> "robotiq_85_base_link"[label=" Broadcaster: default_authority\nAverage rate: 10000.0\nBuffer length: 0.0\nMost recent transform: 0.0\nOldest transform: 0.0\n"];
"robotiq_85_left_knuckle_link" -> "robotiq_85_left_finger_link"[label=" Broadcaster: default_authority\nAverage rate: 10000.0\nBuffer length: 0.0\nMost recent transform: 0.0\nOldest transform: 0.0\n"];
"robotiq_85_right_knuckle_link" -> "robotiq_85_right_finger_link"[label=" Broadcaster: default_authority\nAverage rate: 10000.0\nBuffer length: 0.0\nMost recent transform: 0.0\nOldest transform: 0.0\n"];
"wrist_3_link" -> "ft_frame"[label=" Broadcaster: default_authority\nAverage rate: 10000.0\nBuffer length: 0.0\nMost recent transform: 0.0\nOldest transform: 0.0\n"];
"robotiq_85_base_link" -> "aruco_link"[label=" Broadcaster: default_authority\nAverage rate: 10000.0\nBuffer length: 0.0\nMost recent transform: 0.0\nOldest transform: 0.0\n"];
edge [style=invis];
 subgraph cluster_legend { style=bold; color=black; label ="view_frames Result";
"Recorded at time: 1783507124.8173676"[ shape=plaintext ] ;
}->"wrist_3_link";
}