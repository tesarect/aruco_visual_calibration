#include "aruco_perception/calibration_broadcaster_node.hpp"

#include <chrono>
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

  const auto & waypoints = waypoints_response->waypoints;
  RCLCPP_INFO(get_logger(), "Fetched %zu polygon waypoints", waypoints.size());

  if (!trace_path_client_->wait_for_service(std::chrono::seconds(5))) {
    auto result = std::make_shared<Calibrate::Result>();
    result->success = false;
    result->message = "trajectory_planner's ~/trace_path service not available";
    goal_handle->abort(result);
    RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
    return;
  }

  for (int i = 0; i < config_.num_samples; ++i) {
    if (goal_handle->is_canceling()) {
      auto result = std::make_shared<Calibrate::Result>();
      result->success = false;
      result->message = "Calibration cancelled";
      goal_handle->canceled(result);
      RCLCPP_INFO(get_logger(), "Calibration cancelled");
      return;
    }

    // Cycle through the polygon's corners if num_samples exceeds their
    // count, so a longer run still spreads across the same set of poses
    // rather than failing or stopping early.
    const geometry_msgs::msg::Pose & target = waypoints[i % waypoints.size()];

    auto trace_request = std::make_shared<visual_calibration_msgs::srv::TracePath::Request>();
    trace_request->waypoints = {target};
    trace_request->planning_mode = config_.planning_mode;

    const rclcpp::Time before_move = get_clock()->now();
    auto trace_future = trace_path_client_->async_send_request(trace_request);
    const auto trace_response = trace_future.get();

    if (!trace_response->success) {
      auto result = std::make_shared<Calibrate::Result>();
      result->success = false;
      result->message = "~/trace_path failed for sample " + std::to_string(i + 1) + ": " +
        trace_response->message;
      goal_handle->abort(result);
      RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
      return;
    }

    // The response only arrives once the arm has settled at target (see
    // TrajectoryPlanner::tracePath/planAndExecute[Cartesian]) — that's
    // the settle signal. Still wait for a marker_pose published after
    // this point, rather than trusting whatever was last cached, so the
    // sample can't reflect a frame captured before the arm stopped.
    const std::optional<geometry_msgs::msg::PoseStamped> marker_pose =
      waitForFreshMarkerPose(before_move);

    if (!marker_pose.has_value()) {
      auto result = std::make_shared<Calibrate::Result>();
      result->success = false;
      result->message = "Timed out waiting for a fresh marker_pose for sample " +
        std::to_string(i + 1) + " (is the marker still in view?)";
      goal_handle->abort(result);
      RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
      return;
    }

    if (!recordSample(*marker_pose)) {
      auto result = std::make_shared<Calibrate::Result>();
      result->success = false;
      result->message = "Could not record sample " + std::to_string(i + 1) +
        " (TF lookup failed, see log)";
      goal_handle->abort(result);
      RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
      return;
    }

    RCLCPP_INFO(get_logger(), "Collected sample %d/%d", i + 1, config_.num_samples);

    auto feedback = std::make_shared<Calibrate::Feedback>();
    feedback->samples_collected = static_cast<uint32_t>(collected_positions_.size());
    feedback->samples_total = static_cast<uint32_t>(config_.num_samples);
    goal_handle->publish_feedback(feedback);
  }

  finishCalibration(goal_handle);
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
  broadcast_tf.child_frame_id = last_sample_.header.frame_id;
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
}

CalibrationBroadcasterConfig CalibrationBroadcasterNode::loadConfigFromParams() const
{
  CalibrationBroadcasterConfig config;
  config.marker_pose_topic = get_parameter("marker_pose_topic").as_string();
  config.known_chain_frame = get_parameter("known_chain_frame").as_string();
  config.marker_frame = get_parameter("marker_frame").as_string();
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
  return config;
}

}  // namespace aruco_perception