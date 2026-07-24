#include "aruco_perception/calibration_broadcaster_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace aruco_perception
{

CalibrationBroadcasterNode::CalibrationBroadcasterNode()
: Node(
    "calibration_broadcaster_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
  config_(loadConfigFromParams()),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_),
  static_broadcaster_(this),
  averaging_method_(
    selectAveragingMethod(
      config_.orientation_sum_normalize_priority, config_.orientation_markley_priority))
{
  marker_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    config_.marker_pose_topic, 10,
    std::bind(&CalibrationBroadcasterNode::markerPoseCallback, this, std::placeholders::_1));

  calibrate_action_server_ = rclcpp_action::create_server<Calibrate>(
    this,
    "~/calibrate",
    std::bind(
      &CalibrationBroadcasterNode::handleGoal, this, std::placeholders::_1,
      std::placeholders::_2),
    std::bind(&CalibrationBroadcasterNode::handleCancel, this, std::placeholders::_1),
    std::bind(&CalibrationBroadcasterNode::handleAccepted, this, std::placeholders::_1));

  get_polygon_waypoints_client_ =
    create_client<visual_calibration_msgs::srv::GetPolygonWaypoints>(
    "/trajectory_planner/get_polygon_waypoints");
  trace_path_client_ = create_client<visual_calibration_msgs::srv::TracePath>(
    "/trajectory_planner/trace_path");

  RCLCPP_INFO(
    get_logger(), "calibration_broadcaster_node ready (known_chain_frame: '%s', marker_frame: "
    "'%s', num_samples: %d) — send a ~/calibrate action goal to begin",
    config_.known_chain_frame.c_str(), config_.marker_frame.c_str(), config_.num_samples);
}

void CalibrationBroadcasterNode::markerPoseCallback(
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(sample_mutex_);
  latest_marker_pose_ = *msg;
  latest_marker_pose_stamp_ = get_clock()->now();
  sample_cv_.notify_all();
}

rclcpp_action::GoalResponse CalibrationBroadcasterNode::handleGoal(
  const rclcpp_action::GoalUUID &/*uuid*/,
  std::shared_ptr<const Calibrate::Goal>/*goal*/)
{
  RCLCPP_INFO(get_logger(), "Received ~/calibrate goal");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CalibrationBroadcasterNode::handleCancel(
  const std::shared_ptr<GoalHandleCalibrate>/*goal_handle*/)
{
  RCLCPP_INFO(get_logger(), "Cancelling ~/calibrate goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void CalibrationBroadcasterNode::handleAccepted(
  const std::shared_ptr<GoalHandleCalibrate> goal_handle)
{
  // rclcpp_action requires this callback to return quickly — the actual
  // orchestration (service calls + waiting) runs on its own thread.
  std::thread{
    std::bind(&CalibrationBroadcasterNode::executeCalibration, this, std::placeholders::_1),
    goal_handle}.detach();
}

void CalibrationBroadcasterNode::executeCalibration(
  const std::shared_ptr<GoalHandleCalibrate> goal_handle)
{
  collected_positions_.clear();
  collected_orientations_.clear();
  stable_agreement_count_ = 0;

  if (!get_polygon_waypoints_client_->wait_for_service(std::chrono::seconds(5))) {
    auto result = std::make_shared<Calibrate::Result>();
    result->success = false;
    result->message = "trajectory_planner's ~/get_polygon_waypoints service not available";
    goal_handle->abort(result);
    RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
    return;
  }

  auto waypoints_request =
    std::make_shared<visual_calibration_msgs::srv::GetPolygonWaypoints::Request>();
  auto waypoints_future = get_polygon_waypoints_client_->async_send_request(waypoints_request);
  const auto waypoints_response = waypoints_future.get();

  if (!waypoints_response->success || waypoints_response->waypoints.empty()) {
    auto result = std::make_shared<Calibrate::Result>();
    result->success = false;
    result->message = "Could not fetch polygon waypoints: " + waypoints_response->message;
    goal_handle->abort(result);
    RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
    return;
  }

  const std::vector<geometry_msgs::msg::Pose> waypoints(
    waypoints_response->waypoints.begin(), waypoints_response->waypoints.end());
  const geometry_msgs::msg::Pose center_pose = waypoints_response->center_pose;
  RCLCPP_INFO(get_logger(), "Fetched %zu polygon waypoints", waypoints.size());

  if (!trace_path_client_->wait_for_service(std::chrono::seconds(5))) {
    auto result = std::make_shared<Calibrate::Result>();
    result->success = false;
    result->message = "trajectory_planner's ~/trace_path service not available";
    goal_handle->abort(result);
    RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
    return;
  }

  // Sample once at the center pose itself, right after it's known — the
  // arm is already there (center_pose IS trajectory_planner's own current
  // pose, per polygonWaypointsAroundStandoff's 2026-07-22 redesign), so
  // this needs no additional move, just an immediate marker_pose wait +
  // record before the polygon phase's first corner move begins. Counted
  // toward the same running total as every other sample.
  {
    const rclcpp::Time now = get_clock()->now();
    const std::optional<geometry_msgs::msg::PoseStamped> center_marker_pose =
      waitForFreshMarkerPose(now);

    if (!center_marker_pose.has_value()) {
      auto result = std::make_shared<Calibrate::Result>();
      result->success = false;
      result->message = "Timed out waiting for a fresh marker_pose at the center pose "
        "(is the marker still in view?)";
      goal_handle->abort(result);
      RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
      return;
    }

    if (!recordSample(*center_marker_pose)) {
      auto result = std::make_shared<Calibrate::Result>();
      result->success = false;
      result->message = "Could not record the center-pose sample (TF lookup failed, see log)";
      goal_handle->abort(result);
      RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
      return;
    }

    const int total_samples = 1 + config_.num_samples + config_.random_phase_samples;
    RCLCPP_INFO(get_logger(), "Collected sample 1/%d (center pose)", total_samples);

    auto feedback = std::make_shared<Calibrate::Feedback>();
    feedback->samples_collected = static_cast<uint32_t>(collected_positions_.size());
    feedback->samples_total = static_cast<uint32_t>(total_samples);
    goal_handle->publish_feedback(feedback);
  }

  std::shared_ptr<Calibrate::Result> phase_result;
  bool stopped_early = stableAgreementReached();

  if (!stopped_early && !runPolygonPhase(goal_handle, waypoints, phase_result, stopped_early)) {
    if (phase_result) {
      goal_handle->abort(phase_result);
      RCLCPP_ERROR(get_logger(), "%s", phase_result->message.c_str());
    }
    // else: cancellation already handled (goal_handle->canceled) inside
    // runPolygonPhase itself.
    return;
  }

  if (!stopped_early) {
    if (!runRandomPhase(
        goal_handle, center_pose, static_cast<int>(collected_positions_.size()), phase_result,
        stopped_early))
    {
      if (phase_result) {
        goal_handle->abort(phase_result);
        RCLCPP_ERROR(get_logger(), "%s", phase_result->message.c_str());
      }
      return;
    }
  }

  finishCalibration(goal_handle);
}

bool CalibrationBroadcasterNode::runPolygonPhase(
  const std::shared_ptr<GoalHandleCalibrate> & goal_handle,
  const std::vector<geometry_msgs::msg::Pose> & waypoints,
  std::shared_ptr<Calibrate::Result> & out_result,
  bool & stopped_early)
{
  stopped_early = false;
  const int total_samples = 1 + config_.num_samples + config_.random_phase_samples;

  for (int i = 0; i < config_.num_samples; ++i) {
    if (goal_handle->is_canceling()) {
      auto result = std::make_shared<Calibrate::Result>();
      result->success = false;
      result->message = "Calibration cancelled";
      goal_handle->canceled(result);
      RCLCPP_INFO(get_logger(), "Calibration cancelled");
      out_result = nullptr;
      return false;
    }

    // Cycle through the polygon's corners if num_samples exceeds their
    // count, so a longer run still spreads across the same set of poses
    // rather than failing or stopping early.
    const geometry_msgs::msg::Pose & target = waypoints[i % waypoints.size()];

    // Captured BEFORE the move, not after tracePathBlocking() returns —
    // waitForFreshMarkerPose's whole purpose is rejecting any marker_pose
    // that could have arrived during the move, so the timestamp boundary
    // must predate the move starting, not just predate it settling (a
    // regression risk introduced by extracting tracePathBlocking() as a
    // shared helper during the 2026-07-22 two-phase redesign — fixed
    // here; see error-mitigation.md #19 for why this matters).
    const rclcpp::Time before_move = get_clock()->now();
    if (!tracePathBlocking(target)) {
      out_result = std::make_shared<Calibrate::Result>();
      out_result->success = false;
      out_result->message = "~/trace_path failed for sample " + std::to_string(i + 1);
      return false;
    }

    // The trace_path response only arrives once the arm has settled at
    // target (see TrajectoryPlanner::tracePath/planAndExecute[Cartesian])
    // — that's the settle signal. Still wait for a marker_pose published
    // after before_move, rather than trusting whatever was last cached,
    // so the sample can't reflect a frame captured before the move began.
    const std::optional<geometry_msgs::msg::PoseStamped> marker_pose =
      waitForFreshMarkerPose(before_move);

    if (!marker_pose.has_value()) {
      out_result = std::make_shared<Calibrate::Result>();
      out_result->success = false;
      out_result->message = "Timed out waiting for a fresh marker_pose for sample " +
        std::to_string(i + 1) + " (is the marker still in view?)";
      return false;
    }

    if (!recordSample(*marker_pose)) {
      out_result = std::make_shared<Calibrate::Result>();
      out_result->success = false;
      out_result->message = "Could not record sample " + std::to_string(i + 1) +
        " (TF lookup failed, see log)";
      return false;
    }

    RCLCPP_INFO(
      get_logger(), "Collected sample %zu/%d (polygon phase)", collected_positions_.size(),
      total_samples);

    auto feedback = std::make_shared<Calibrate::Feedback>();
    feedback->samples_collected = static_cast<uint32_t>(collected_positions_.size());
    feedback->samples_total = static_cast<uint32_t>(total_samples);
    goal_handle->publish_feedback(feedback);

    if (stableAgreementReached()) {
      RCLCPP_INFO(
        get_logger(), "Early-stop: agreement reached after %zu samples (polygon phase)",
        collected_positions_.size());
      stopped_early = true;
      return true;
    }
  }

  return true;
}

bool CalibrationBroadcasterNode::runRandomPhase(
  const std::shared_ptr<GoalHandleCalibrate> & goal_handle,
  const geometry_msgs::msg::Pose & center_pose,
  int samples_already_collected,
  std::shared_ptr<Calibrate::Result> & out_result,
  bool & stopped_early)
{
  stopped_early = false;
  const int total_samples = 1 + config_.num_samples + config_.random_phase_samples;
  int consecutive_failures = 0;

  for (int i = 0; i < config_.random_phase_samples; ) {
    if (goal_handle->is_canceling()) {
      auto result = std::make_shared<Calibrate::Result>();
      result->success = false;
      result->message = "Calibration cancelled";
      goal_handle->canceled(result);
      RCLCPP_INFO(get_logger(), "Calibration cancelled");
      out_result = nullptr;
      return false;
    }

    const geometry_msgs::msg::Pose candidate =
      randomPoseNear(center_pose, config_.random_phase_max_offset_m);

    // Captured BEFORE the move — see runPolygonPhase's identical comment
    // on why this must predate the move starting, not just its settling.
    const rclcpp::Time before_move = get_clock()->now();
    if (!tracePathBlocking(candidate)) {
      // A failed move (e.g. planAndExecuteCartesian refusing an
      // incomplete straight-line path — see trajectory_planner.cpp's
      // cartesian_min_fraction check) is treated the same as an
      // invisible-marker candidate: discarded, not counted, retried with
      // a new random candidate, bounded by the same consecutive-failure
      // cap (2026-07-23 — a random offset can point in any direction, so
      // occasionally landing on one direction's Cartesian-path limit is
      // expected, not a reason to abort an otherwise-successful run). No
      // return-to-center needed here — planAndExecuteCartesian refuses
      // BEFORE calling execute() on an incomplete path, so the arm never
      // actually moved; it's already still at the last good pose.
      ++consecutive_failures;
      RCLCPP_INFO(
        get_logger(), "Random-phase candidate's move failed (attempt %d/%d consecutive) — "
        "trying a new candidate", consecutive_failures,
        config_.random_phase_max_consecutive_failures);

      if (consecutive_failures >= config_.random_phase_max_consecutive_failures) {
        out_result = std::make_shared<Calibrate::Result>();
        out_result->success = false;
        out_result->message = "Random phase gave up after " +
          std::to_string(consecutive_failures) + " consecutive failed/invisible candidates";
        return false;
      }
      continue;
    }

    if (!isMarkerVisibleNow(before_move)) {
      // Discarded, not counted — return to center immediately (no point
      // probing further out when not visible at all here) and try a new
      // candidate.
      ++consecutive_failures;
      RCLCPP_INFO(
        get_logger(), "Random-phase candidate not visible (attempt %d/%d consecutive) — "
        "returning to center", consecutive_failures, config_.random_phase_max_consecutive_failures);

      if (consecutive_failures >= config_.random_phase_max_consecutive_failures) {
        out_result = std::make_shared<Calibrate::Result>();
        out_result->success = false;
        out_result->message = "Random phase gave up after " +
          std::to_string(consecutive_failures) + " consecutive failed/invisible candidates";
        return false;
      }

      if (!tracePathBlocking(center_pose)) {
        out_result = std::make_shared<Calibrate::Result>();
        out_result->success = false;
        out_result->message = "Could not return to center pose after an invisible random candidate";
        return false;
      }
      continue;
    }

    consecutive_failures = 0;

    const std::optional<geometry_msgs::msg::PoseStamped> marker_pose =
      waitForFreshMarkerPose(before_move);
    if (!marker_pose.has_value()) {
      out_result = std::make_shared<Calibrate::Result>();
      out_result->success = false;
      out_result->message = "Timed out waiting for a fresh marker_pose for random-phase sample " +
        std::to_string(samples_already_collected + i + 1);
      return false;
    }

    if (!recordSample(*marker_pose)) {
      out_result = std::make_shared<Calibrate::Result>();
      out_result->success = false;
      out_result->message = "Could not record random-phase sample " +
        std::to_string(samples_already_collected + i + 1) + " (TF lookup failed, see log)";
      return false;
    }

    ++i;
    RCLCPP_INFO(
      get_logger(), "Collected sample %zu/%d (random phase)", collected_positions_.size(),
      total_samples);

    auto feedback = std::make_shared<Calibrate::Feedback>();
    feedback->samples_collected = static_cast<uint32_t>(collected_positions_.size());
    feedback->samples_total = static_cast<uint32_t>(total_samples);
    goal_handle->publish_feedback(feedback);

    if (stableAgreementReached()) {
      RCLCPP_INFO(
        get_logger(), "Early-stop: agreement reached after %zu samples (random phase)",
        collected_positions_.size());
      stopped_early = true;
      return true;
    }
  }

  return true;
}

geometry_msgs::msg::Pose CalibrationBroadcasterNode::randomPoseNear(
  const geometry_msgs::msg::Pose & center_pose, double max_offset_m) const
{
  // Uniform offset within a cube of side 2*max_offset_m, re-rolled until
  // its straight-line distance from center is within max_offset_m (a
  // simple rejection sampler — cheap at this scale, no closed-form
  // uniform-sphere sampling needed).
  std::uniform_real_distribution<double> axis_dist(-max_offset_m, max_offset_m);

  double dx = 0.0;
  double dy = 0.0;
  double dz = 0.0;
  do {
    dx = axis_dist(random_engine_);
    dy = axis_dist(random_engine_);
    dz = axis_dist(random_engine_);
  } while (std::sqrt(dx * dx + dy * dy + dz * dz) > max_offset_m);

  tf2::Transform center;
  tf2::fromMsg(center_pose, center);
  const tf2::Transform offset(tf2::Quaternion::getIdentity(), tf2::Vector3(dx, dy, dz));
  const tf2::Transform result = center * offset;

  geometry_msgs::msg::Pose result_pose;
  result_pose.position.x = result.getOrigin().x();
  result_pose.position.y = result.getOrigin().y();
  result_pose.position.z = result.getOrigin().z();
  result_pose.orientation = tf2::toMsg(result.getRotation());
  return result_pose;
}

bool CalibrationBroadcasterNode::tracePathBlocking(const geometry_msgs::msg::Pose & target)
{
  auto trace_request = std::make_shared<visual_calibration_msgs::srv::TracePath::Request>();
  trace_request->waypoints = {target};
  trace_request->planning_mode = config_.planning_mode;

  auto trace_future = trace_path_client_->async_send_request(trace_request);
  const auto trace_response = trace_future.get();
  return trace_response->success;
}

std::optional<geometry_msgs::msg::PoseStamped> CalibrationBroadcasterNode::waitForFreshMarkerPose(
  const rclcpp::Time & after)
{
  std::unique_lock<std::mutex> lock(sample_mutex_);
  const bool got_fresh_sample = sample_cv_.wait_for(
    lock, std::chrono::duration<double>(config_.sample_wait_timeout_sec),
    [this, &after]() {
      return latest_marker_pose_stamp_.nanoseconds() > 0 && latest_marker_pose_stamp_ > after;
    });

  if (!got_fresh_sample) {
    return std::nullopt;
  }
  return latest_marker_pose_;
}

bool CalibrationBroadcasterNode::isMarkerVisibleNow(const rclcpp::Time & after)
{
  // Polls rather than blocking on the condition variable (unlike
  // waitForFreshMarkerPose) — a random-phase candidate at a position
  // where the marker is genuinely out of view will never produce a fresh
  // message at all, so this has to time out gracefully, not hang. Same
  // pattern as CalibrationOrchestratorNode::isMarkerVisibleAfter.
  const rclcpp::Time deadline =
    get_clock()->now() + rclcpp::Duration::from_seconds(config_.sample_wait_timeout_sec);

  while (get_clock()->now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(sample_mutex_);
      if (latest_marker_pose_stamp_.nanoseconds() > 0 && latest_marker_pose_stamp_ > after) {
        return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return false;
}

bool CalibrationBroadcasterNode::recordSample(
  const geometry_msgs::msg::PoseStamped & marker_pose)
{
  // marker_pose is camera_frame -> marker_frame (camera's own frame_id,
  // marker's pose within it). Invert to get marker_frame -> camera_frame,
  // then chain with the known known_chain_frame -> marker_frame TF to get
  // one sample of known_chain_frame -> camera_frame.
  tf2::Transform camera_to_marker;
  tf2::fromMsg(marker_pose.pose, camera_to_marker);
  const tf2::Transform marker_to_camera = camera_to_marker.inverse();

  geometry_msgs::msg::TransformStamped known_to_marker_tf;
  try {
    known_to_marker_tf = tf_buffer_.lookupTransform(
      config_.known_chain_frame, config_.marker_frame, tf2::TimePointZero,
      tf2::durationFromSec(1.0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(
      get_logger(), "Could not look up '%s' -> '%s': %s",
      config_.known_chain_frame.c_str(), config_.marker_frame.c_str(), ex.what());
    return false;
  }

  tf2::Transform known_to_marker;
  tf2::fromMsg(known_to_marker_tf.transform, known_to_marker);
  const tf2::Transform known_to_camera = known_to_marker * marker_to_camera;

  geometry_msgs::msg::Vector3 sample_position;
  sample_position.x = known_to_camera.getOrigin().x();
  sample_position.y = known_to_camera.getOrigin().y();
  sample_position.z = known_to_camera.getOrigin().z();
  collected_positions_.push_back(sample_position);
  collected_orientations_.push_back(known_to_camera.getRotation());

  last_sample_.header = marker_pose.header;
  last_sample_.pose.position.x = sample_position.x;
  last_sample_.pose.position.y = sample_position.y;
  last_sample_.pose.position.z = sample_position.z;
  last_sample_.pose.orientation = tf2::toMsg(known_to_camera.getRotation());

  return true;
}

bool CalibrationBroadcasterNode::stableAgreementReached()
{
  const size_t count = collected_positions_.size();
  if (count < 2) {
    // Spread is meaningless with only 0/1 samples collected.
    return false;
  }

  geometry_msgs::msg::Vector3 mean_position;
  for (const geometry_msgs::msg::Vector3 & position : collected_positions_) {
    mean_position.x += position.x;
    mean_position.y += position.y;
    mean_position.z += position.z;
  }
  mean_position.x /= static_cast<double>(count);
  mean_position.y /= static_cast<double>(count);
  mean_position.z /= static_cast<double>(count);

  double max_position_spread_m = 0.0;
  for (const geometry_msgs::msg::Vector3 & position : collected_positions_) {
    const double dx = position.x - mean_position.x;
    const double dy = position.y - mean_position.y;
    const double dz = position.z - mean_position.z;
    max_position_spread_m = std::max(max_position_spread_m, std::sqrt(dx * dx + dy * dy + dz * dz));
  }
  const double max_position_spread_cm = max_position_spread_m * 100.0;

  const OrientationAveragingResult orientation_result =
    averageQuaternions(collected_orientations_, averaging_method_);

  const bool within_tolerance =
    max_position_spread_cm <= config_.position_spread_tolerance_cm &&
    orientation_result.max_spread_deg <= config_.orientation_spread_tolerance_deg;

  if (within_tolerance) {
    ++stable_agreement_count_;
    RCLCPP_INFO(
      get_logger(), "Early-stop check: sample within tolerance (position spread %.2fcm, "
      "orientation spread %.2fdeg) — agreement count %d/%d",
      max_position_spread_cm, orientation_result.max_spread_deg, stable_agreement_count_,
      config_.stable_agreement_count);
  }
  // Deliberately NOT reset on a single out-of-tolerance sample — see this
  // method's doc comment (a running, non-consecutive count).

  return stable_agreement_count_ >= config_.stable_agreement_count;
}

void CalibrationBroadcasterNode::finishCalibration(
  const std::shared_ptr<GoalHandleCalibrate> & goal_handle)
{
  geometry_msgs::msg::Vector3 average_position;
  for (const geometry_msgs::msg::Vector3 & position : collected_positions_) {
    average_position.x += position.x;
    average_position.y += position.y;
    average_position.z += position.z;
  }
  const double count = static_cast<double>(collected_positions_.size());
  average_position.x /= count;
  average_position.y /= count;
  average_position.z /= count;

  const OrientationAveragingResult orientation_result =
    averageQuaternions(collected_orientations_, averaging_method_);

  geometry_msgs::msg::TransformStamped broadcast_tf;
  broadcast_tf.header.stamp = get_clock()->now();
  broadcast_tf.header.frame_id = config_.known_chain_frame;
  // Suffixed, not the detector's raw frame_id: broadcasting under the
  // exact same name as an existing URDF-declared frame (e.g. sim's
  // wrist_rgbd_camera_depth_optical_frame) would conflict with it in the
  // TF tree — two disagreeing publishers for one frame. See
  // CalibrationBroadcasterConfig::broadcast_frame_suffix.
  broadcast_tf.child_frame_id = last_sample_.header.frame_id + config_.broadcast_frame_suffix;
  broadcast_tf.transform.translation = average_position;
  broadcast_tf.transform.rotation = tf2::toMsg(orientation_result.averaged);

  static_broadcaster_.sendTransform(broadcast_tf);

  RCLCPP_INFO(
    get_logger(), "Calibration complete: broadcasting static TF '%s' -> '%s' "
    "(position + orientation averaged over %zu samples; orientation spread: "
    "max %.3f deg, mean %.3f deg)",
    config_.known_chain_frame.c_str(), broadcast_tf.child_frame_id.c_str(),
    collected_positions_.size(), orientation_result.max_spread_deg,
    orientation_result.mean_spread_deg);

  auto result = std::make_shared<Calibrate::Result>();
  result->success = true;
  result->message = "Broadcasting static TF '" + config_.known_chain_frame + "' -> '" +
    broadcast_tf.child_frame_id + "'";
  result->max_spread_deg = orientation_result.max_spread_deg;
  result->mean_spread_deg = orientation_result.mean_spread_deg;
  goal_handle->succeed(result);

  collected_positions_.clear();
  collected_orientations_.clear();
  stable_agreement_count_ = 0;
}

CalibrationBroadcasterConfig CalibrationBroadcasterNode::loadConfigFromParams() const
{
  CalibrationBroadcasterConfig config;
  config.marker_pose_topic = get_parameter("marker_pose_topic").as_string();
  config.known_chain_frame = get_parameter("known_chain_frame").as_string();
  config.marker_frame = get_parameter("marker_frame").as_string();
  config.broadcast_frame_suffix = get_parameter("broadcast_frame_suffix").as_string();
  config.num_samples = static_cast<int>(get_parameter("num_samples").as_int());
  config.sample_wait_timeout_sec = get_parameter("sample_wait_timeout_sec").as_double();

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

  config.orientation_sum_normalize_priority =
    static_cast<int>(get_parameter("orientation_sum_normalize_priority").as_int());
  config.orientation_markley_priority =
    static_cast<int>(get_parameter("orientation_markley_priority").as_int());

  config.random_phase_samples =
    static_cast<int>(get_parameter("random_phase_samples").as_int());
  config.random_phase_max_offset_m = get_parameter("random_phase_max_offset_m").as_double();
  config.random_phase_max_consecutive_failures =
    static_cast<int>(get_parameter("random_phase_max_consecutive_failures").as_int());

  config.position_spread_tolerance_cm =
    get_parameter("position_spread_tolerance_cm").as_double();
  config.orientation_spread_tolerance_deg =
    get_parameter("orientation_spread_tolerance_deg").as_double();
  config.stable_agreement_count =
    static_cast<int>(get_parameter("stable_agreement_count").as_int());

  return config;
}

}  // namespace aruco_perception