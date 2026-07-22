#include "orchestrator/calibration_orchestrator_node.hpp"

#include <array>
#include <chrono>
#include <stdexcept>
#include <thread>
#include <utility>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace orchestrator
{

CalibrationOrchestratorNode::CalibrationOrchestratorNode()
: Node(
    "calibration_orchestrator_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
  config_(loadConfigFromParams())
{
  marker_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/aruco_perception/marker_pose", 10,
    std::bind(&CalibrationOrchestratorNode::markerPoseCallback, this, std::placeholders::_1));

  auto_calibrate_action_server_ = rclcpp_action::create_server<AutoCalibrate>(
    this,
    "~/auto_calibrate",
    std::bind(
      &CalibrationOrchestratorNode::handleGoal, this, std::placeholders::_1,
      std::placeholders::_2),
    std::bind(&CalibrationOrchestratorNode::handleCancel, this, std::placeholders::_1),
    std::bind(&CalibrationOrchestratorNode::handleAccepted, this, std::placeholders::_1));

  calibrate_action_client_ = rclcpp_action::create_client<Calibrate>(
    this, "/calibration_broadcaster_node/calibrate");

  get_standoff_pose_client_ =
    create_client<visual_calibration_msgs::srv::GetStandoffPose>(
    "/trajectory_planner/get_standoff_pose");
  trace_path_client_ = create_client<visual_calibration_msgs::srv::TracePath>(
    "/trajectory_planner/trace_path");
  move_to_preset_client_ = create_client<visual_calibration_msgs::srv::MoveToPreset>(
    "/trajectory_planner/move_to_preset");

  // See handleStartAutoCalibrate's doc comment — rosbridge-reachable
  // facade in front of ~/auto_calibrate, since rosbridge_suite 1.3.1 (this
  // project's version) has no ROS2 action support at all.
  self_action_client_ = rclcpp_action::create_client<AutoCalibrate>(this, "~/auto_calibrate");
  start_auto_calibrate_service_ = create_service<std_srvs::srv::Trigger>(
    "~/start_auto_calibrate",
    std::bind(
      &CalibrationOrchestratorNode::handleStartAutoCalibrate, this, std::placeholders::_1,
      std::placeholders::_2));
  cancel_auto_calibrate_service_ = create_service<std_srvs::srv::Trigger>(
    "~/cancel_auto_calibrate",
    std::bind(
      &CalibrationOrchestratorNode::handleCancelAutoCalibrate, this, std::placeholders::_1,
      std::placeholders::_2));
  auto_calibrate_status_pub_ = create_publisher<visual_calibration_msgs::msg::AutoCalibrateStatus>(
    "~/auto_calibrate_status", rclcpp::QoS(10).reliable());

  // classical_detector_param_client_/hybrid_detector_param_client_ are
  // deliberately NOT constructed here — see their doc comment in the
  // header (shared_from_this() isn't safe yet during this constructor).
  set_detector_mode_service_ = create_service<visual_calibration_msgs::srv::SetDetectorMode>(
    "~/set_detector_mode",
    std::bind(
      &CalibrationOrchestratorNode::handleSetDetectorMode, this, std::placeholders::_1,
      std::placeholders::_2));

  RCLCPP_INFO(
    get_logger(),
    "calibration_orchestrator_node ready (auto_center_enabled: %s) — send a "
    "~/auto_calibrate action goal to begin",
    config_.auto_center_enabled ? "true" : "false");
}

void CalibrationOrchestratorNode::markerPoseCallback(
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr &/*msg*/)
{
  std::lock_guard<std::mutex> lock(marker_mutex_);
  latest_marker_pose_stamp_ = get_clock()->now();
}

void CalibrationOrchestratorNode::publishStatusFeedback(const AutoCalibrate::Feedback & feedback)
{
  auto status = visual_calibration_msgs::msg::AutoCalibrateStatus();
  status.phase = visual_calibration_msgs::msg::AutoCalibrateStatus::PHASE_RUNNING;
  status.stage = feedback.stage;
  status.samples_collected = feedback.samples_collected;
  status.samples_total = feedback.samples_total;
  auto_calibrate_status_pub_->publish(status);
}

void CalibrationOrchestratorNode::publishStatusResult(const AutoCalibrate::Result & result)
{
  auto status = visual_calibration_msgs::msg::AutoCalibrateStatus();
  status.phase = result.success ?
    visual_calibration_msgs::msg::AutoCalibrateStatus::PHASE_SUCCEEDED :
    visual_calibration_msgs::msg::AutoCalibrateStatus::PHASE_FAILED;
  status.success = result.success;
  status.message = result.message;
  status.max_spread_deg = result.max_spread_deg;
  status.mean_spread_deg = result.mean_spread_deg;
  status.failed_stage = result.failed_stage;
  auto_calibrate_status_pub_->publish(status);
}

void CalibrationOrchestratorNode::handleStartAutoCalibrate(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!self_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    response->success = false;
    response->message = "~/auto_calibrate action server not available";
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

  auto goal = AutoCalibrate::Goal();

  rclcpp_action::Client<AutoCalibrate>::SendGoalOptions options;
  options.goal_response_callback =
    [this](const rclcpp_action::ClientGoalHandle<AutoCalibrate>::SharedPtr & goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(get_logger(), "~/auto_calibrate goal was rejected by the action server");
        return;
      }
      std::lock_guard<std::mutex> lock(self_action_goal_handle_mutex_);
      self_action_goal_handle_ = goal_handle;
    };
  options.feedback_callback =
    [this](
    rclcpp_action::ClientGoalHandle<AutoCalibrate>::SharedPtr/*goal_handle*/,
    const std::shared_ptr<const AutoCalibrate::Feedback> feedback) {
      publishStatusFeedback(*feedback);
    };
  options.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<AutoCalibrate>::WrappedResult & wrapped_result) {
      if (wrapped_result.result) {
        publishStatusResult(*wrapped_result.result);
        return;
      }
      // No result object at all (e.g. the goal was aborted/canceled by the
      // client library itself before the server ever produced one) —
      // still publish SOMETHING so a subscriber waiting on this topic
      // doesn't hang indefinitely with no terminal message.
      auto result = AutoCalibrate::Result();
      result.success = false;
      result.message = "~/auto_calibrate goal did not complete normally (no result)";
      result.failed_stage = "";
      publishStatusResult(result);
    };

  // Fire-and-forget from THIS service call's perspective — the caller
  // (e.g. the web app) gets success=true here once the goal is merely
  // submitted, then watches ~/auto_calibrate_status for actual progress/
  // completion. Not waited on with .get() here deliberately: that would
  // block this service callback for the full multi-minute calibration
  // duration, which — unlike an action server — a plain ROS2 service has
  // no built-in mechanism for a client to await asynchronously without
  // blocking its own executor thread.
  self_action_client_->async_send_goal(goal, options);

  response->success = true;
  response->message = "~/auto_calibrate goal submitted — watch ~/auto_calibrate_status for progress";
}

void CalibrationOrchestratorNode::handleCancelAutoCalibrate(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  rclcpp_action::ClientGoalHandle<AutoCalibrate>::SharedPtr goal_handle;
  {
    std::lock_guard<std::mutex> lock(self_action_goal_handle_mutex_);
    goal_handle = self_action_goal_handle_;
  }

  if (!goal_handle) {
    response->success = false;
    response->message = "No ~/auto_calibrate goal has been started yet — nothing to cancel";
    return;
  }

  self_action_client_->async_cancel_goal(goal_handle);
  response->success = true;
  response->message = "Cancel request sent for the current ~/auto_calibrate goal";
}

rclcpp::SyncParametersClient::SharedPtr
CalibrationOrchestratorNode::getClassicalDetectorParamClient()
{
  // shared_from_this() is only safe once this node is already owned by a
  // shared_ptr (true by the time any service callback runs, e.g. now) —
  // see the member's doc comment in the header for why this can't happen
  // in the constructor. Lazily built once, reused after.
  if (!classical_detector_param_client_) {
    classical_detector_param_client_ = std::make_shared<rclcpp::SyncParametersClient>(
      shared_from_this(), "aruco_detector_node");
  }
  return classical_detector_param_client_;
}

rclcpp::SyncParametersClient::SharedPtr
CalibrationOrchestratorNode::getHybridDetectorParamClient()
{
  if (!hybrid_detector_param_client_) {
    hybrid_detector_param_client_ = std::make_shared<rclcpp::SyncParametersClient>(
      shared_from_this(), "yolo_marker_bridge_node");
  }
  return hybrid_detector_param_client_;
}

void CalibrationOrchestratorNode::handleSetDetectorMode(
  const std::shared_ptr<visual_calibration_msgs::srv::SetDetectorMode::Request> request,
  std::shared_ptr<visual_calibration_msgs::srv::SetDetectorMode::Response> response)
{
  bool classical_should_be_active;
  if (request->mode == "classical") {
    classical_should_be_active = true;
  } else if (request->mode == "hybrid") {
    classical_should_be_active = false;
  } else {
    response->success = false;
    response->message = "Unknown mode '" + request->mode + "' — must be 'classical' or 'hybrid'";
    return;
  }

  auto classical_client = getClassicalDetectorParamClient();
  auto hybrid_client = getHybridDetectorParamClient();

  static constexpr auto kServiceWaitTimeout = std::chrono::seconds(2);
  if (!classical_client->wait_for_service(kServiceWaitTimeout)) {
    response->success = false;
    response->message = "aruco_detector_node's parameter service is not reachable "
      "(is it running?) — no changes made";
    return;
  }
  if (!hybrid_client->wait_for_service(kServiceWaitTimeout)) {
    response->success = false;
    response->message = "yolo_marker_bridge_node's parameter service is not reachable "
      "(is it running?) — no changes made";
    return;
  }

  // Set the node coming ONLINE first, then the one going offline — briefly
  // both-active (one extra duplicate marker_pose sample, harmless) is
  // preferable to briefly neither-active (a real, if brief, gap in the
  // pose stream) — see header doc comment.
  const auto & incoming_client = classical_should_be_active ? classical_client : hybrid_client;
  const auto & outgoing_client = classical_should_be_active ? hybrid_client : classical_client;
  const char * incoming_name = classical_should_be_active ?
    "aruco_detector_node" : "yolo_marker_bridge_node";
  const char * outgoing_name = classical_should_be_active ?
    "yolo_marker_bridge_node" : "aruco_detector_node";

  // set_parameters_atomically (one SetParametersResult, not a vector) is
  // the better fit here over plain set_parameters — each call only ever
  // sets the single "active" parameter, so there's nothing for
  // "atomically" to buy us beyond a simpler return type to check.
  const auto incoming_result = incoming_client->set_parameters_atomically(
    {rclcpp::Parameter("active", true)});
  if (!incoming_result.successful) {
    response->success = false;
    response->message = std::string("Failed to activate ") + incoming_name + ": " +
      incoming_result.reason;
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

  const auto outgoing_result = outgoing_client->set_parameters_atomically(
    {rclcpp::Parameter("active", false)});
  if (!outgoing_result.successful) {
    // Inconsistent state: incoming is now active but outgoing failed to
    // deactivate — both nodes would publish marker_pose simultaneously
    // until this is retried. Not silently swallowed — logged loudly and
    // reported as a failure so a caller knows to retry/investigate, even
    // though the incoming switch itself did succeed.
    response->success = false;
    response->message = std::string("Activated ") + incoming_name +
      " but FAILED to deactivate " + outgoing_name + ": " + outgoing_result.reason +
      " — both detectors may now be active simultaneously, retry this call";
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

  response->success = true;
  response->message = std::string("Switched to ") + request->mode + " (" + incoming_name +
    " active, " + outgoing_name + " inactive)";
  RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}

rclcpp_action::GoalResponse CalibrationOrchestratorNode::handleGoal(
  const rclcpp_action::GoalUUID &/*uuid*/,
  std::shared_ptr<const AutoCalibrate::Goal>/*goal*/)
{
  RCLCPP_INFO(get_logger(), "Received ~/auto_calibrate goal");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CalibrationOrchestratorNode::handleCancel(
  const std::shared_ptr<GoalHandleAutoCalibrate>/*goal_handle*/)
{
  RCLCPP_INFO(get_logger(), "Cancelling ~/auto_calibrate goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void CalibrationOrchestratorNode::handleAccepted(
  const std::shared_ptr<GoalHandleAutoCalibrate> goal_handle)
{
  std::thread{
    std::bind(&CalibrationOrchestratorNode::executeAutoCalibrate, this, std::placeholders::_1),
    goal_handle}.detach();
}

void CalibrationOrchestratorNode::executeAutoCalibrate(
  const std::shared_ptr<GoalHandleAutoCalibrate> goal_handle)
{
  auto publish_stage = [this, &goal_handle](const std::string & stage) {
      auto feedback = std::make_shared<AutoCalibrate::Feedback>();
      feedback->stage = stage;
      goal_handle->publish_feedback(feedback);
    };

  auto abort_with = [this, &goal_handle](const std::string & failed_stage, const std::string & message) {
      auto result = std::make_shared<AutoCalibrate::Result>();
      result->success = false;
      result->message = message;
      result->failed_stage = failed_stage;
      goal_handle->abort(result);
      RCLCPP_ERROR(get_logger(), "%s", message.c_str());
    };

  // Stage 1: move to cal_ready/standoff.
  publish_stage("Moving to cal_ready");
  const std::optional<geometry_msgs::msg::Pose> cal_ready_pose = moveToCalReady();
  if (!cal_ready_pose.has_value()) {
    abort_with("cal_ready", "Could not move to cal_ready/standoff pose (see log)");
    return;
  }

  if (goal_handle->is_canceling()) {
    auto result = std::make_shared<AutoCalibrate::Result>();
    result->success = false;
    result->message = "Cancelled after reaching cal_ready";
    result->failed_stage = "";
    goal_handle->canceled(result);
    return;
  }

  std::this_thread::sleep_for(
    std::chrono::duration<double>(config_.post_cal_ready_settle_seconds));

  // Stage 3 (only if enabled — stage 2's "wait" is the sleep above).
  // Read live via get_parameter(), NOT config_.auto_center_enabled — this
  // field is documented (see OrchestratorConfig's doc comment) as
  // web-toggleable at runtime via a standard ~/set_parameters call with no
  // restart, but config_ itself is only ever populated once, in the
  // constructor's loadConfigFromParams() call — nothing refreshes it
  // afterward, so the cached copy would silently ignore any runtime
  // toggle. This is the one field in config_ that genuinely needs a fresh
  // read per run; the rest of config_ is legitimately load-once (session
  // settle/probe/timeout tuning that isn't meant to change mid-session).
  if (get_parameter("auto_center_enabled").as_bool()) {
    publish_stage("Auto-centering on marker");
    const std::optional<geometry_msgs::msg::Pose> centered_pose =
      runAutoCenterProbe(*cal_ready_pose);
    if (!centered_pose.has_value()) {
      abort_with("auto_center", "Auto-center probe failed (see log)");
      return;
    }
  }

  if (goal_handle->is_canceling()) {
    auto result = std::make_shared<AutoCalibrate::Result>();
    result->success = false;
    result->message = "Cancelled before calibration started";
    result->failed_stage = "";
    goal_handle->canceled(result);
    return;
  }

  // Stage 4: calibrate.
  publish_stage("Calibrating");
  const std::shared_ptr<Calibrate::Result> calibrate_result = runCalibrate(goal_handle);
  if (!calibrate_result) {
    abort_with("calibrate", "calibration_broadcaster_node's ~/calibrate action not reachable");
    return;
  }

  auto result = std::make_shared<AutoCalibrate::Result>();
  result->success = calibrate_result->success;
  result->message = calibrate_result->message;
  result->max_spread_deg = calibrate_result->max_spread_deg;
  result->mean_spread_deg = calibrate_result->mean_spread_deg;
  result->failed_stage = calibrate_result->success ? "" : "calibrate";

  if (calibrate_result->success) {
    goal_handle->succeed(result);
  } else {
    goal_handle->abort(result);
    RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
  }
}

std::optional<geometry_msgs::msg::Pose> CalibrationOrchestratorNode::moveToCalReady()
{
  // A previous run's auto-centering already found a better center for THIS
  // session — reuse it instead of re-deriving cal_ready from scratch
  // (which would drift back to the original, possibly off-marker-center
  // pose; see session_centered_cal_ready_pose_'s doc comment). Still
  // physically moves the arm there via tracePathBlocking, same as the
  // joint-preset/TF-based paths below — only the SOURCE of the target pose
  // changes.
  if (session_centered_cal_ready_pose_.has_value()) {
    RCLCPP_INFO(get_logger(), "Moving to this session's previously auto-centered cal_ready pose.");
    if (!tracePathBlocking(*session_centered_cal_ready_pose_, "cal_ready")) {
      return std::nullopt;
    }
    return session_centered_cal_ready_pose_;
  }

  // Try the joint-value preset path FIRST — see this method's header
  // comment for why (pins the IK branch, avoids the ~80-83%-partial
  // Cartesian polygon-corner failures a bad branch can cause downstream).
  // "No 'cal_ready' preset is configured" (from TrajectoryPlanner::
  // planAndExecuteToPreset, relayed via MoveToPreset::Response::message)
  // is treated as "fall through to the original TF-based path", not an
  // error — every OTHER move_to_preset failure (e.g. planning genuinely
  // failing on a configured joint preset) is treated as a real failure,
  // matching the original method's error-handling strictness.
  bool moved_via_joint_preset = false;
  if (move_to_preset_client_->wait_for_service(std::chrono::seconds(5))) {
    auto preset_request = std::make_shared<visual_calibration_msgs::srv::MoveToPreset::Request>();
    preset_request->name = "cal_ready";
    auto preset_future = move_to_preset_client_->async_send_request(preset_request);
    const auto preset_response = preset_future.get();

    if (preset_response->success) {
      RCLCPP_INFO(get_logger(), "Moved to cal_ready via its joint-value preset.");
      moved_via_joint_preset = true;
    } else if (preset_response->message.find("No preset named") == std::string::npos) {
      // A configured preset exists but planning/execution genuinely
      // failed — a real failure, not a "fall through" case.
      RCLCPP_ERROR(
        get_logger(), "~/move_to_preset(cal_ready) failed: %s",
        preset_response->message.c_str());
      return std::nullopt;
    }
    // else: no "cal_ready" preset configured at all — fall through below.
  } else {
    RCLCPP_WARN(
      get_logger(),
      "trajectory_planner's ~/move_to_preset service not available — falling back to "
      "~/get_standoff_pose directly.");
  }

  if (!get_standoff_pose_client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(get_logger(), "trajectory_planner's ~/get_standoff_pose service not available");
    return std::nullopt;
  }

  auto standoff_request = std::make_shared<visual_calibration_msgs::srv::GetStandoffPose::Request>();
  auto standoff_future = get_standoff_pose_client_->async_send_request(standoff_request);
  const auto standoff_response = standoff_future.get();

  if (!standoff_response->success) {
    RCLCPP_ERROR(
      get_logger(), "Could not compute standoff pose: %s", standoff_response->message.c_str());
    return std::nullopt;
  }

  // Already moved via the joint preset above — ~/get_standoff_pose here is
  // ONLY a read (no motion, see TrajectoryPlanner::getStandoffPose) to
  // learn the resulting Cartesian pose for runAutoCenterProbe; do NOT also
  // send ~/trace_path, which would move the arm a second time.
  if (!moved_via_joint_preset) {
    if (!tracePathBlocking(standoff_response->standoff_pose, "cal_ready")) {
      return std::nullopt;
    }
  }

  return standoff_response->standoff_pose;
}

bool CalibrationOrchestratorNode::isMarkerVisibleAfter(const rclcpp::Time & after)
{
  // Poll rather than block-and-wait on a condition variable — a probe move
  // to a position where the marker is genuinely out of view will never
  // produce a fresh message at all, so this has to time out gracefully,
  // not hang. A short sleep loop is simple and fine at this timescale
  // (probe moves themselves take much longer than the poll interval).
  const rclcpp::Time deadline =
    get_clock()->now() + rclcpp::Duration::from_seconds(config_.auto_center_visibility_timeout_sec);

  while (get_clock()->now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(marker_mutex_);
      if (latest_marker_pose_stamp_.nanoseconds() > 0 && latest_marker_pose_stamp_ > after) {
        return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return false;
}

bool CalibrationOrchestratorNode::probeDirectionVisible(
  const geometry_msgs::msg::Pose & center_pose,
  double x_axis, double y_axis, double distance_m)
{
  tf2::Transform center;
  tf2::fromMsg(center_pose, center);

  // Offset applied in center_pose's own local X/Y plane, same convention
  // as TrajectoryPlanner::polygonWaypointsAroundStandoff's corner offsets
  // — orientation is kept identical to center_pose, only position moves.
  const tf2::Transform offset(
    tf2::Quaternion::getIdentity(),
    tf2::Vector3(x_axis * distance_m, y_axis * distance_m, 0.0));
  const tf2::Transform probe = center * offset;

  geometry_msgs::msg::Pose probe_pose;
  probe_pose.position.x = probe.getOrigin().x();
  probe_pose.position.y = probe.getOrigin().y();
  probe_pose.position.z = probe.getOrigin().z();
  probe_pose.orientation = tf2::toMsg(probe.getRotation());

  const rclcpp::Time before_move = get_clock()->now();
  if (!tracePathBlocking(probe_pose)) {
    // A failed plan/execute (e.g. near a joint limit) is treated the same
    // as "marker not visible" — either way this direction can't go
    // further, so the caller should stop extending it here.
    return false;
  }

  return isMarkerVisibleAfter(before_move);
}

namespace
{
/// Applies (offset_x, offset_y) to base_pose's own local X/Y plane —
/// shared by runAutoCenterProbe's per-axis recentering steps below.
geometry_msgs::msg::Pose offsetInLocalPlane(
  const geometry_msgs::msg::Pose & base_pose, double offset_x, double offset_y)
{
  tf2::Transform base;
  tf2::fromMsg(base_pose, base);
  const tf2::Transform offset(
    tf2::Quaternion::getIdentity(), tf2::Vector3(offset_x, offset_y, 0.0));
  const tf2::Transform result = base * offset;

  geometry_msgs::msg::Pose result_pose;
  result_pose.position.x = result.getOrigin().x();
  result_pose.position.y = result.getOrigin().y();
  result_pose.position.z = result.getOrigin().z();
  result_pose.orientation = tf2::toMsg(result.getRotation());
  return result_pose;
}
}  // namespace

std::optional<geometry_msgs::msg::Pose> CalibrationOrchestratorNode::runAutoCenterProbe(
  const geometry_msgs::msg::Pose & center_pose)
{
  // Axis-by-axis recentering: probe +X/-X from center_pose, move to the
  // X-midpoint BEFORE probing Y at all, then probe +Y/-Y from that
  // already-X-centered pose. This differs from probing all 4 directions
  // from the original center_pose and correcting both axes in one final
  // move (the original design) — axis-by-axis means the Y probe starts
  // from a pose that's already corrected on X, so its own boundaries
  // reflect the marker's true visibility extent along Y at the right X
  // position, rather than at the original (possibly off-center) X.
  const std::array<std::pair<double, double>, 2> x_directions = {{
      {1.0, 0.0},    // +X
      {-1.0, 0.0},   // -X
    }};
  std::array<double, 2> x_boundaries_m = {0.0, 0.0};
  for (size_t d = 0; d < x_directions.size(); ++d) {
    const auto [x_axis, y_axis] = x_directions[d];
    double reached_m = 0.0;
    double distance_m = config_.auto_center_probe_step_m;
    while (distance_m <= config_.auto_center_max_probe_m) {
      if (!probeDirectionVisible(center_pose, x_axis, y_axis, distance_m)) {
        break;
      }
      reached_m = distance_m;
      distance_m += config_.auto_center_probe_step_m;
    }
    x_boundaries_m[d] = reached_m;
    RCLCPP_INFO(
      get_logger(), "Auto-center X probe direction %zu (x=%.0f): boundary at %.3fm",
      d, x_axis, reached_m);
  }

  const double center_offset_x = (x_boundaries_m[0] - x_boundaries_m[1]) / 2.0;
  const geometry_msgs::msg::Pose x_centered_pose =
    offsetInLocalPlane(center_pose, center_offset_x, 0.0);

  RCLCPP_INFO(
    get_logger(), "Auto-center: moving to X-centered pose (offset x=%.3fm)", center_offset_x);
  if (!tracePathBlocking(x_centered_pose, "cal_ready")) {
    return std::nullopt;
  }

  const std::array<std::pair<double, double>, 2> y_directions = {{
      {0.0, 1.0},    // +Y
      {0.0, -1.0},   // -Y
    }};
  std::array<double, 2> y_boundaries_m = {0.0, 0.0};
  for (size_t d = 0; d < y_directions.size(); ++d) {
    const auto [x_axis, y_axis] = y_directions[d];
    double reached_m = 0.0;
    double distance_m = config_.auto_center_probe_step_m;
    while (distance_m <= config_.auto_center_max_probe_m) {
      if (!probeDirectionVisible(x_centered_pose, x_axis, y_axis, distance_m)) {
        break;
      }
      reached_m = distance_m;
      distance_m += config_.auto_center_probe_step_m;
    }
    y_boundaries_m[d] = reached_m;
    RCLCPP_INFO(
      get_logger(), "Auto-center Y probe direction %zu (y=%.0f): boundary at %.3fm",
      d, y_axis, reached_m);
  }

  const double center_offset_y = (y_boundaries_m[0] - y_boundaries_m[1]) / 2.0;
  const geometry_msgs::msg::Pose corrected_pose =
    offsetInLocalPlane(x_centered_pose, 0.0, center_offset_y);

  RCLCPP_INFO(
    get_logger(), "Auto-center: moving to Y-centered pose (offset y=%.3fm)", center_offset_y);
  if (!tracePathBlocking(corrected_pose, "cal_ready")) {
    return std::nullopt;
  }

  // Persist for this session — see session_centered_cal_ready_pose_'s doc
  // comment (moveToCalReady reuses this on a later run instead of
  // re-deriving cal_ready from scratch).
  session_centered_cal_ready_pose_ = corrected_pose;

  return corrected_pose;
}

bool CalibrationOrchestratorNode::tracePathBlocking(
  const geometry_msgs::msg::Pose & target,
  const std::string & pose_name)
{
  if (!trace_path_client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(get_logger(), "trajectory_planner's ~/trace_path service not available");
    return false;
  }

  auto request = std::make_shared<visual_calibration_msgs::srv::TracePath::Request>();
  request->waypoints = {target};
  request->planning_mode = config_.planning_mode;
  request->pose_name = pose_name;
  request->is_sequenced_goal = false;

  auto future = trace_path_client_->async_send_request(request);
  const auto response = future.get();

  if (!response->success) {
    RCLCPP_ERROR(get_logger(), "~/trace_path failed: %s", response->message.c_str());
    return false;
  }
  return true;
}

std::shared_ptr<CalibrationOrchestratorNode::Calibrate::Result>
CalibrationOrchestratorNode::runCalibrate(
  const std::shared_ptr<GoalHandleAutoCalibrate> & goal_handle)
{
  if (!calibrate_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    return nullptr;
  }

  auto send_goal_options = rclcpp_action::Client<Calibrate>::SendGoalOptions();
  send_goal_options.feedback_callback =
    [this, &goal_handle](
    rclcpp_action::ClientGoalHandle<Calibrate>::SharedPtr /*handle*/,
    const std::shared_ptr<const Calibrate::Feedback> feedback) {
      auto relayed = std::make_shared<AutoCalibrate::Feedback>();
      relayed->stage = "Calibrating";
      relayed->samples_collected = feedback->samples_collected;
      relayed->samples_total = feedback->samples_total;
      goal_handle->publish_feedback(relayed);
    };

  auto goal_handle_future =
    calibrate_action_client_->async_send_goal(Calibrate::Goal(), send_goal_options);
  const auto calibrate_goal_handle = goal_handle_future.get();
  if (!calibrate_goal_handle) {
    RCLCPP_ERROR(get_logger(), "~/calibrate goal was rejected");
    return nullptr;
  }

  auto result_future = calibrate_action_client_->async_get_result(calibrate_goal_handle);
  const auto wrapped_result = result_future.get();

  // A CANCELED/ABORTED wrapped_result still carries a valid ->result with
  // success=false and a message — pass it through as-is rather than
  // synthesizing a generic one, so the caller sees calibration_broadcaster_
  // node's actual reason.
  return wrapped_result.result;
}

OrchestratorConfig CalibrationOrchestratorNode::loadConfigFromParams() const
{
  OrchestratorConfig config;
  config.post_cal_ready_settle_seconds =
    get_parameter("post_cal_ready_settle_seconds").as_double();
  config.auto_center_enabled = get_parameter("auto_center_enabled").as_bool();
  config.auto_center_probe_step_m = get_parameter("auto_center_probe_step_m").as_double();
  config.auto_center_max_probe_m = get_parameter("auto_center_max_probe_m").as_double();
  config.auto_center_visibility_timeout_sec =
    get_parameter("auto_center_visibility_timeout_sec").as_double();

  const std::string mode_name = get_parameter("planning_mode").as_string();
  if (mode_name == "cartesian") {
    config.planning_mode = visual_calibration_msgs::srv::TracePath::Request::PLANNING_MODE_CARTESIAN;
  } else if (mode_name == "joint_space") {
    config.planning_mode =
      visual_calibration_msgs::srv::TracePath::Request::PLANNING_MODE_JOINT_SPACE;
  } else {
    throw std::invalid_argument(
            "Unknown planning_mode: '" + mode_name + "' (expected 'cartesian' or 'joint_space')");
  }

  return config;
}

}  // namespace orchestrator