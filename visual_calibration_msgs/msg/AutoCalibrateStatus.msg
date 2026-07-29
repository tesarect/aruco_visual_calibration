# Published on calibration_orchestrator_node's ~/auto_calibrate_status
# topic, one message per AutoCalibrate.action feedback/result event —
# mirrors that action's Feedback/Result fields exactly, but reachable from
# clients that can't speak rosbridge's native ROS2 action protocol (e.g.
# the web app: rosbridge_suite 1.3.1's allowed operations list has NO
# action support at all — confirmed via rosbridge's own rejection message,
# "Unknown operation: send_action_goal. Allowed operations: ['call_service',
# 'advertise', 'unadvertise', 'publish', 'subscribe', 'unsubscribe',
# 'fragment', 'advertise_service', 'service_response',
# 'unadvertise_service']" — 2026-07-20). Start the sequence via
# ~/start_auto_calibrate (std_srvs/Trigger) instead of sending an action
# goal directly, then subscribe to this topic for progress/result. The
# underlying ~/auto_calibrate action is unchanged and still directly usable
# by any ROS2-native client (e.g. `ros2 action send_goal`) — this topic is
# an additive facade, not a replacement.

uint8 PHASE_RUNNING = 0
uint8 PHASE_SUCCEEDED = 1
uint8 PHASE_FAILED = 2
uint8 phase

# Feedback fields — meaningful while phase == PHASE_RUNNING.
string stage
uint32 samples_collected
uint32 samples_total
# Relayed from AutoCalibrate.action's own latest_sample_pose feedback field
# (2026-07-29) — see Calibrate.action's latest_sample_pose doc comment for
# the full rationale. Zero-valued (default Pose) for every non-
# "Calibrating" stage feedback.
geometry_msgs/Pose latest_sample_pose

# Result fields — meaningful once phase != PHASE_RUNNING (zero-valued
# while running). Mirrors AutoCalibrate.action's Result exactly.
bool success
string message
float64 max_spread_deg
float64 mean_spread_deg
string failed_stage
# Relayed from AutoCalibrate.action's own is_high_confidence result field
# (2026-07-29) — see Calibrate.action's is_high_confidence doc comment for
# the full rationale.
bool is_high_confidence