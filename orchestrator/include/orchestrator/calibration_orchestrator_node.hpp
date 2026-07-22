#ifndef ORCHESTRATOR__CALIBRATION_ORCHESTRATOR_NODE_HPP_
#define ORCHESTRATOR__CALIBRATION_ORCHESTRATOR_NODE_HPP_

#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visual_calibration_msgs/action/auto_calibrate.hpp>
#include <visual_calibration_msgs/action/calibrate.hpp>
#include <visual_calibration_msgs/msg/auto_calibrate_status.hpp>
#include <visual_calibration_msgs/srv/get_standoff_pose.hpp>
#include <visual_calibration_msgs/srv/move_to_preset.hpp>
#include <visual_calibration_msgs/srv/set_detector_mode.hpp>
#include <visual_calibration_msgs/srv/trace_path.hpp>

namespace orchestrator
{

/// Tuning for CalibrationOrchestratorNode, loaded from a parameter file.
/// Neither trajectory_planner nor calibration_broadcaster_node are told
/// this orchestration exists — trajectory_planner only ever sees ordinary
/// ~/get_standoff_pose/~/trace_path calls (same calls a human/web operator
/// would issue by hand today), and calibration_broadcaster_node only ever
/// sees an ordinary ~/calibrate goal. All new sequencing logic lives here.
struct OrchestratorConfig
{
  /// How long to wait after reaching cal_ready/standoff before starting
  /// auto-centering (if enabled) or calibration — gives the camera a
  /// moment to settle beyond trajectory_planner's own per-waypoint
  /// waypoint_settle_seconds (that one already fires inside the trace_path
  /// call this node makes; this is an *additional* pause specific to the
  /// "about to start a multi-minute calibration run" moment, kept
  /// separately tunable rather than reusing trajectory_planner's param).
  double post_cal_ready_settle_seconds = 2.0;
  /// Master switch for the auto-centering safety step (see
  /// runAutoCenterProbe) — default false. Overridable at runtime via a
  /// standard ROS set_parameters call (same pattern the web app already
  /// uses for calibration_broadcaster_node's planning_mode), so the web
  /// app can flip this without a code change or restart. NOTE: this field
  /// of config_ is only used for the startup ready-log line — the actual
  /// per-run check in executeAutoCalibrate() reads get_parameter(
  /// "auto_center_enabled") live instead of this cached copy, specifically
  /// so a runtime toggle takes effect on the very next ~/auto_calibrate
  /// run without needing loadConfigFromParams() to re-run (fixed
  /// 2026-07-22 — this field used to be read directly and silently
  /// ignored any post-startup toggle, since nothing ever refreshed it).
  bool auto_center_enabled = false;
  /// Distance (meters) moved per probe step, in the standoff pose's own
  /// local X/Y plane (same convention as trajectory_planner's
  /// polygon_radius_m), when searching for each of the four +X/-X/+Y/-Y
  /// visibility boundaries.
  double auto_center_probe_step_m = 0.05;
  /// Maximum probe distance (meters) from cal_ready along any single axis
  /// direction before giving up on that direction and treating cal_ready's
  /// own position as the boundary — keeps the probe search bounded even if
  /// the marker never disappears (e.g. a very wide camera FOV).
  double auto_center_max_probe_m = 0.15;
  /// How long to wait for a marker_pose message after each probe move
  /// before concluding the marker is not visible there.
  double auto_center_visibility_timeout_sec = 2.0;
  /// Which planning mode to request on every ~/trace_path call this node
  /// makes (cal_ready move + auto-center probe moves) — see
  /// TracePath::Request::PLANNING_MODE_*.
  uint8_t planning_mode =
    visual_calibration_msgs::srv::TracePath::Request::PLANNING_MODE_JOINT_SPACE;
};

/// Orchestrates the full auto-calibrate sequence behind one action,
/// ~/auto_calibrate:
///   1. Move to cal_ready/standoff (trajectory_planner's
///      ~/get_standoff_pose, then ~/trace_path with the result — the exact
///      two calls the web app's "Cal Ready" button already makes today).
///   2. Wait post_cal_ready_settle_seconds.
///   3. If auto_center_enabled: probe +X/-X/+Y/-Y from cal_ready (see
///      runAutoCenterProbe) to find how far the marker stays visible in
///      each direction, then move to the midpoint of the surviving
///      boundaries — trajectory_planner's own ~/get_standoff_pose/polygon
///      math re-centers on wherever the arm is now, since it always
///      recomputes the standoff fresh from the live camera TF (see
///      TrajectoryPlanner::polygonWaypointsAroundStandoff) — no new API
///      needed on trajectory_planner's side to "tell" it about the
///      corrected center.
///   4. Call calibration_broadcaster_node's existing ~/calibrate action as
///      a client, relaying its feedback/result through this action.
///
/// Neither trajectory_planner nor calibration_broadcaster_node change to
/// support this — this node only ever calls their already-existing public
/// interfaces, the same ones a human/web operator could call by hand.
///
/// Also exposes ~/start_auto_calibrate (std_srvs/Trigger) +
/// ~/auto_calibrate_status (visual_calibration_msgs/AutoCalibrateStatus,
/// plain topic) as a rosbridge-reachable facade in front of the
/// ~/auto_calibrate action above — see handleStartAutoCalibrate's doc
/// comment for why (rosbridge_suite 1.3.1, this project's version, has NO
/// ROS2 action support at all — confirmed 2026-07-20). The action itself
/// is unchanged and still directly usable by any ROS2-native client (e.g.
/// `ros2 action send_goal`); the facade is additive, not a replacement.
///
/// Also exposes ~/set_detector_mode (visual_calibration_msgs/
/// SetDetectorMode) — the classical/hybrid detector switch. Neither
/// aruco_perception's ArucoDetectorNode nor aruco_perception_yolo_bridge's
/// YoloMarkerBridgeNode know this orchestration exists, same philosophy as
/// the rest of this class: this node just flips one node's "active"
/// parameter true and the other's false via two rclcpp::SyncParametersClient
/// instances (one per detector node) and the standard ROS set_parameters
/// service — no lifecycle nodes, no process start/stop, both detector
/// nodes stay running/subscribed the whole time. See handleSetDetectorMode.
class CalibrationOrchestratorNode : public rclcpp::Node
{
public:
  using AutoCalibrate = visual_calibration_msgs::action::AutoCalibrate;
  using GoalHandleAutoCalibrate = rclcpp_action::ServerGoalHandle<AutoCalibrate>;
  using Calibrate = visual_calibration_msgs::action::Calibrate;

  CalibrationOrchestratorNode();

private:
  OrchestratorConfig loadConfigFromParams() const;

  /// Caches the latest marker_pose message's receipt time — used only by
  /// runAutoCenterProbe to check "is the marker visible right now", not
  /// for calibration sampling (calibration_broadcaster_node subscribes to
  /// the same topic independently for that).
  void markerPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr & msg);

  /// Handles ~/start_auto_calibrate (std_srvs/Trigger) — a rosbridge-
  /// reachable facade in front of ~/auto_calibrate, for clients that can't
  /// speak rosbridge's native ROS2 action protocol (see
  /// AutoCalibrateStatus.msg's header comment: rosbridge_suite 1.3.1 has
  /// NO action support at all, confirmed 2026-07-20 via its own
  /// "Unknown operation: send_action_goal" rejection). Sends a normal
  /// AutoCalibrate goal to THIS node's own action server via
  /// self_action_client_ (a supported pattern — a node can be a client of
  /// its own action server), relays feedback/result onto
  /// ~/auto_calibrate_status (see publishStatus*), and returns
  /// success=true immediately once the goal is ACCEPTED — this is
  /// fire-and-forget from the service caller's perspective, not
  /// fire-and-wait-for-completion; progress/result only ever arrive via
  /// the status topic. Returns success=false (Trigger's response) only if
  /// goal submission itself failed (e.g. action server unreachable) —
  /// never if the calibration sequence itself later fails, since that's
  /// reported via the status topic instead.
  void handleStartAutoCalibrate(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /// Handles ~/cancel_auto_calibrate (std_srvs/Trigger) — the cancel-side
  /// half of the same rosbridge facade as handleStartAutoCalibrate (see
  /// its doc comment). Cancels the goal tracked in
  /// self_action_goal_handle_ (set once the goal-response callback in
  /// handleStartAutoCalibrate confirms acceptance) via
  /// self_action_client_->async_cancel_goal(goal_handle) — see
  /// rclcpp_action::Client. Returns success=false if there is no in-flight
  /// goal to cancel (nothing started yet, or the previous run already
  /// finished) — this is a legitimate "nothing to do" outcome, not
  /// treated as a server error.
  void handleCancelAutoCalibrate(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /// Handles ~/set_detector_mode. request.mode must be "classical" or
  /// "hybrid" (anything else is rejected with success=false, response.mode
  /// left unset — a validation failure, not a node-unreachable failure).
  /// Sets active=true on the target node's SyncParametersClient FIRST,
  /// THEN active=false on the other — briefly both-active rather than
  /// briefly neither-active, since a duplicate marker_pose publish for one
  /// frame is harmless (calibration_broadcaster_node just sees an extra
  /// sample) while a gap where NEITHER publishes is a real, if brief, loss
  /// of the pose stream. Returns success=false (with a descriptive
  /// response.message) if either set_parameters call fails or times out —
  /// does NOT partially apply the switch: if the second call fails after
  /// the first succeeded, logs the resulting inconsistent state rather
  /// than silently leaving it unclear which detector a caller should
  /// believe is authoritative.
  void handleSetDetectorMode(
    const std::shared_ptr<visual_calibration_msgs::srv::SetDetectorMode::Request> request,
    std::shared_ptr<visual_calibration_msgs::srv::SetDetectorMode::Response> response);

  /// Publishes one AutoCalibrateStatus with phase=PHASE_RUNNING and the
  /// given feedback fields (result fields left zero-valued) — called from
  /// self_action_client_'s feedback callback, mirroring exactly what
  /// publish_stage's lambda inside executeAutoCalibrate already sends as
  /// native action feedback (see that function) — this is a parallel
  /// broadcast, not a replacement.
  void publishStatusFeedback(const AutoCalibrate::Feedback & feedback);

  /// Publishes one AutoCalibrateStatus with phase=PHASE_SUCCEEDED or
  /// PHASE_FAILED (per result.success) and the given result fields
  /// (feedback fields left zero-valued) — called from
  /// self_action_client_'s result callback.
  void publishStatusResult(const AutoCalibrate::Result & result);

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const AutoCalibrate::Goal> goal);

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleAutoCalibrate> goal_handle);

  /// Spawns a detached thread running executeAutoCalibrate(goal_handle) —
  /// rclcpp_action requires handleAccepted to return quickly, not block.
  void handleAccepted(const std::shared_ptr<GoalHandleAutoCalibrate> goal_handle);

  /// The actual sequence, run on its own thread — see class doc comment
  /// for the 4 stages. Aborts (goal_handle->abort, with failed_stage set)
  /// on the first stage that fails.
  void executeAutoCalibrate(const std::shared_ptr<GoalHandleAutoCalibrate> goal_handle);

  /// Stage 1: tries trajectory_planner's ~/move_to_preset("cal_ready")
  /// FIRST — if a joint-value preset named "cal_ready" is configured (see
  /// preset_poses_sim.yaml/_real.yaml, TrajectoryPlanner::planAndExecuteToPreset),
  /// this pins the exact IK branch/joint configuration instead of leaving
  /// it to whatever solution a fresh ~/get_standoff_pose + ~/trace_path
  /// call happens to land on. Confirmed 2026-07-20: two different
  /// joint-space paths to the SAME cal_ready Cartesian pose produced joint
  /// configurations differing by 90-250° on several joints, only one of
  /// which left enough margin for the downstream Cartesian polygon-corner
  /// moves (runAutoCenterProbe, calibration_broadcaster_node's sampling)
  /// to succeed rather than fail partway (computeCartesianPath() stopping
  /// at ~80-83%).
  /// Falls back to the ORIGINAL behavior — ~/get_standoff_pose then
  /// ~/trace_path with the result (pose_name "cal_ready", is_sequenced_goal
  /// false — same call shape the web app's "Cal Ready" button already
  /// makes) — if move_to_preset reports no "cal_ready" preset is configured
  /// (e.g. real, which has no joint-value preset captured yet as of
  /// 2026-07-20, only its existing Cartesian one). Either way, returns the
  /// resulting Cartesian pose on success (queried via ~/get_standoff_pose
  /// after a successful joint-preset move — that call is read-only/
  /// no-motion, see TrajectoryPlanner::getStandoffPose — since
  /// runAutoCenterProbe needs the actual Cartesian pose regardless of which
  /// path got the arm there), or std::nullopt on failure (logs the reason).
  std::optional<geometry_msgs::msg::Pose> moveToCalReady();

  /// Returns true if a marker_pose message was received within
  /// config_.auto_center_visibility_timeout_sec of `after`.
  bool isMarkerVisibleAfter(const rclcpp::Time & after);

  /// Stage 3: axis-by-axis recentering from center_pose (cal_ready's
  /// pose). First probes +X/-X in center_pose's own local X/Y plane in
  /// config_.auto_center_probe_step_m increments (each probe: move via
  /// ~/trace_path, then isMarkerVisibleAfter) until the marker disappears
  /// or config_.auto_center_max_probe_m is reached — that distance becomes
  /// the boundary for that direction (0.0 if the marker wasn't even
  /// visible one step out). Moves to the X-midpoint pose BEFORE probing Y
  /// at all, so the Y probe's own boundaries reflect visibility at the
  /// already-X-corrected position, not the original (possibly off-center)
  /// X. Then repeats the same probe+move pattern for +Y/-Y from that
  /// X-centered pose, landing on the final X-and-Y-centered pose.
  /// Orientation is kept identical to center_pose throughout — only
  /// position changes. On success, also stores the result in
  /// session_centered_cal_ready_pose_ (see its doc comment) so a later
  /// moveToCalReady() call in the same session reuses it instead of
  /// re-deriving cal_ready from scratch. Returns the corrected pose moved
  /// to on success, std::nullopt if any of the required moves failed
  /// outright (NOT if a probe simply finds a 0.0 boundary — a marker not
  /// visible even one step out just means that direction's boundary is
  /// the starting pose itself, which is a valid, if degenerate, result).
  std::optional<geometry_msgs::msg::Pose> runAutoCenterProbe(
    const geometry_msgs::msg::Pose & center_pose);

  /// One probe move + visibility check in the given local-frame direction
  /// (+1/-1 on x_axis xor y_axis, not both) at `distance_m` from
  /// center_pose. Returns true if the marker was visible there. Does not
  /// move back to center_pose — callers issue the next probe or the final
  /// centering move themselves.
  bool probeDirectionVisible(
    const geometry_msgs::msg::Pose & center_pose,
    double x_axis, double y_axis, double distance_m);

  /// Sends a single-waypoint ~/trace_path request and blocks for the
  /// response — shared by moveToCalReady/runAutoCenterProbe. pose_name/
  /// is_sequenced_goal forwarded as given (both default to the "no
  /// bookkeeping" values used by every caller in this file). Returns
  /// nullopt if the service isn't available or the call fails.
  bool tracePathBlocking(
    const geometry_msgs::msg::Pose & target,
    const std::string & pose_name = "");

  /// Stage 4: sends an empty goal to calibration_broadcaster_node's
  /// ~/calibrate action, relaying its feedback as AutoCalibrate feedback
  /// (stage "Calibrating (sample N/M)") and blocking until it completes.
  /// Returns the Calibrate::Result on success or failure alike (check
  /// ->success) — nullopt only if the action server itself wasn't
  /// reachable.
  std::shared_ptr<Calibrate::Result> runCalibrate(
    const std::shared_ptr<GoalHandleAutoCalibrate> & goal_handle);

  OrchestratorConfig config_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr marker_pose_sub_;
  rclcpp_action::Server<AutoCalibrate>::SharedPtr auto_calibrate_action_server_;
  rclcpp_action::Client<Calibrate>::SharedPtr calibrate_action_client_;
  /// This node's own client of its own ~/auto_calibrate action server —
  /// see handleStartAutoCalibrate's doc comment for why (rosbridge-facade
  /// support). A perfectly normal, supported rclcpp_action pattern; not
  /// circular in any problematic sense — goal submission/feedback/result
  /// all go through the same executor as any other client would see.
  rclcpp_action::Client<AutoCalibrate>::SharedPtr self_action_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_auto_calibrate_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_auto_calibrate_service_;
  /// See handleSetDetectorMode. Lazily constructed on first use (inside
  /// getClassicalDetectorParamClient/getHybridDetectorParamClient), NOT in
  /// this node's own constructor — rclcpp::SyncParametersClient's
  /// constructor requires a std::shared_ptr<Node>, obtained here via
  /// shared_from_this(), which throws if called before this node object is
  /// itself already wrapped in a shared_ptr by whoever created it (true by
  /// the time any service callback runs, NOT true during this node's own
  /// constructor body).
  rclcpp::SyncParametersClient::SharedPtr classical_detector_param_client_;
  rclcpp::SyncParametersClient::SharedPtr hybrid_detector_param_client_;
  rclcpp::SyncParametersClient::SharedPtr getClassicalDetectorParamClient();
  rclcpp::SyncParametersClient::SharedPtr getHybridDetectorParamClient();
  rclcpp::Service<visual_calibration_msgs::srv::SetDetectorMode>::SharedPtr
    set_detector_mode_service_;
  /// The in-flight goal from the most recent ~/start_auto_calibrate call,
  /// set once its goal-response callback confirms server-side acceptance
  /// — used by handleCancelAutoCalibrate. Guarded by
  /// self_action_goal_handle_mutex_ since the goal-response callback
  /// (executor thread) and handleCancelAutoCalibrate (a service callback,
  /// possibly a different executor thread under MultiThreadedExecutor)
  /// both touch it. Left non-null after completion (not reset to nullptr
  /// on result) — a cancel call against an already-finished goal is a
  /// harmless no-op via rclcpp_action's own handling, so there's no need
  /// to track "is this goal still active" separately here.
  rclcpp_action::ClientGoalHandle<AutoCalibrate>::SharedPtr self_action_goal_handle_;
  std::mutex self_action_goal_handle_mutex_;
  /// See AutoCalibrateStatus.msg's header comment — plain reliable QoS
  /// (not transient_local): a caller must ~/start_auto_calibrate first,
  /// THEN subscribe, to see this run's progress; there is no meaningful
  /// "last status" to replay to a late subscriber across separate runs.
  rclcpp::Publisher<visual_calibration_msgs::msg::AutoCalibrateStatus>::SharedPtr
    auto_calibrate_status_pub_;
  rclcpp::Client<visual_calibration_msgs::srv::GetStandoffPose>::SharedPtr
    get_standoff_pose_client_;
  rclcpp::Client<visual_calibration_msgs::srv::TracePath>::SharedPtr trace_path_client_;
  rclcpp::Client<visual_calibration_msgs::srv::MoveToPreset>::SharedPtr move_to_preset_client_;
  /// The most recent auto-centered pose (set by runAutoCenterProbe on
  /// success), if any — treated as this session's effective cal_ready
  /// until the arm is moved elsewhere or a fresh ~/auto_calibrate run
  /// re-probes. In-memory only (not written back to preset_poses_*.yaml):
  /// lost on node restart, same as any other runtime-only state in this
  /// class. moveToCalReady() returns this instead of re-querying
  /// ~/get_standoff_pose when set, so a second Calibrate press in the same
  /// session reuses the corrected center rather than drifting back to the
  /// original (possibly off-marker-center) cal_ready pose.
  std::optional<geometry_msgs::msg::Pose> session_centered_cal_ready_pose_;

  /// Guards latest_marker_pose_stamp_, notified by markerPoseCallback and
  /// read by isMarkerVisibleAfter. Deliberately separate from
  /// calibration_broadcaster_node's own equivalent state — this node only
  /// needs "was anything received recently", not the pose value itself.
  std::mutex marker_mutex_;
  rclcpp::Time latest_marker_pose_stamp_;
};

}  // namespace orchestrator

#endif  // ORCHESTRATOR__CALIBRATION_ORCHESTRATOR_NODE_HPP_