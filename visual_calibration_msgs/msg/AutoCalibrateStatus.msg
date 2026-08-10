# Published on calibration_orchestrator_node's ~/auto_calibrate_status
# topic, one message per AutoCalibrate.action feedback/result event —
# mirrors that action's Feedback/Result fields exactly, but reachable from
# clients that can't speak rosbridge's native ROS2 action protocol (e.g.
# the web app: rosbridge_suite's allowed operations list has no action
# support at all). Start the sequence via ~/start_auto_calibrate
# (std_srvs/Trigger) instead of sending an action goal directly, then
# subscribe to this topic for progress/result. The underlying
# ~/auto_calibrate action is unchanged and still directly usable by any
# ROS2-native client (e.g. `ros2 action send_goal`) — this topic is an
# additive facade, not a replacement.

uint8 PHASE_RUNNING = 0
uint8 PHASE_SUCCEEDED = 1
uint8 PHASE_FAILED = 2
uint8 phase

# Feedback fields — meaningful while phase == PHASE_RUNNING.
string stage
uint32 samples_collected
uint32 samples_total
# Relayed from AutoCalibrate.action's own latest_sample_pose feedback field
# — see Calibrate.action's latest_sample_pose doc comment for the full
# rationale. Zero-valued (default Pose) for every non-"Calibrating" stage
# feedback.
geometry_msgs/Pose latest_sample_pose
# Relayed from AutoCalibrate.action's own current_status feedback field —
# see Calibrate.action's current_status doc comment for the full
# rationale. Empty except for discard-and-continue/timeout events.
string current_status

# Result fields — meaningful once phase != PHASE_RUNNING (zero-valued
# while running). Mirrors AutoCalibrate.action's Result exactly.
bool success
string message
float64 max_spread_deg
float64 mean_spread_deg
string failed_stage
# Relayed from AutoCalibrate.action's own is_high_confidence result field
# — see Calibrate.action's is_high_confidence doc comment for the full
# rationale.
bool is_high_confidence