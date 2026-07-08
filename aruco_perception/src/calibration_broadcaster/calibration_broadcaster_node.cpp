#include "aruco_perception/calibration_broadcaster_node.hpp"

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
  static_broadcaster_(this)
{
  marker_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    config_.marker_pose_topic, 10,
    std::bind(&CalibrationBroadcasterNode::markerPoseCallback, this, std::placeholders::_1));

  start_calibration_service_ = create_service<std_srvs::srv::Trigger>(
    "~/start_calibration",
    std::bind(
      &CalibrationBroadcasterNode::handleStartCalibration, this, std::placeholders::_1,
      std::placeholders::_2));

  RCLCPP_INFO(
    get_logger(), "calibration_broadcaster_node ready (known_chain_frame: '%s', marker_frame: "
    "'%s', num_samples: %d) — call ~/start_calibration to begin",
    config_.known_chain_frame.c_str(), config_.marker_frame.c_str(), config_.num_samples);
}

void CalibrationBroadcasterNode::markerPoseCallback(
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr & msg)
{
  if (!is_collecting_) {
    return;
  }

  const rclcpp::Time now = get_clock()->now();
  if (collected_positions_.size() > 0 &&
    (now - last_sample_time_).seconds() < config_.min_sample_interval_sec)
  {
    return;
  }

  // msg is camera_frame -> marker_frame (camera's own frame_id, marker's
  // pose within it). Invert to get marker_frame -> camera_frame, then
  // chain with the known known_chain_frame -> marker_frame TF to get one
  // sample of known_chain_frame -> camera_frame.
  tf2::Transform camera_to_marker;
  tf2::fromMsg(msg->pose, camera_to_marker);
  const tf2::Transform marker_to_camera = camera_to_marker.inverse();

  geometry_msgs::msg::TransformStamped known_to_marker_tf;
  try {
    known_to_marker_tf = tf_buffer_.lookupTransform(
      config_.known_chain_frame, config_.marker_frame, tf2::TimePointZero,
      tf2::durationFromSec(1.0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "Could not look up '%s' -> '%s': %s",
      config_.known_chain_frame.c_str(), config_.marker_frame.c_str(), ex.what());
    return;
  }

  tf2::Transform known_to_marker;
  tf2::fromMsg(known_to_marker_tf.transform, known_to_marker);
  const tf2::Transform known_to_camera = known_to_marker * marker_to_camera;

  geometry_msgs::msg::Vector3 sample_position;
  sample_position.x = known_to_camera.getOrigin().x();
  sample_position.y = known_to_camera.getOrigin().y();
  sample_position.z = known_to_camera.getOrigin().z();
  collected_positions_.push_back(sample_position);

  last_sample_.header = msg->header;
  last_sample_.pose.position.x = sample_position.x;
  last_sample_.pose.position.y = sample_position.y;
  last_sample_.pose.position.z = sample_position.z;
  last_sample_.pose.orientation = tf2::toMsg(known_to_camera.getRotation());

  last_sample_time_ = now;

  RCLCPP_INFO(
    get_logger(), "Collected sample %zu/%d", collected_positions_.size(),
    config_.num_samples);

  if (static_cast<int>(collected_positions_.size()) >= config_.num_samples) {
    finishCalibration();
  }
}

void CalibrationBroadcasterNode::handleStartCalibration(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (is_collecting_) {
    response->success = false;
    response->message = "Calibration already in progress";
    return;
  }

  collected_positions_.clear();
  is_collecting_ = true;

  response->success = true;
  response->message = "Calibration started — collecting " +
    std::to_string(config_.num_samples) + " samples";
  RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}

void CalibrationBroadcasterNode::finishCalibration()
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

  geometry_msgs::msg::TransformStamped broadcast_tf;
  broadcast_tf.header.stamp = get_clock()->now();
  broadcast_tf.header.frame_id = config_.known_chain_frame;
  broadcast_tf.child_frame_id = last_sample_.header.frame_id;
  broadcast_tf.transform.translation = average_position;
  // Orientation not yet averaged — see class doc.
  broadcast_tf.transform.rotation = last_sample_.pose.orientation;

  static_broadcaster_.sendTransform(broadcast_tf);

  RCLCPP_INFO(
    get_logger(), "Calibration complete: broadcasting static TF '%s' -> '%s' "
    "(position averaged over %zu samples, orientation from most recent sample)",
    config_.known_chain_frame.c_str(), broadcast_tf.child_frame_id.c_str(),
    collected_positions_.size());

  collected_positions_.clear();
  is_collecting_ = false;
}

CalibrationBroadcasterConfig CalibrationBroadcasterNode::loadConfigFromParams() const
{
  CalibrationBroadcasterConfig config;
  config.marker_pose_topic = get_parameter("marker_pose_topic").as_string();
  config.known_chain_frame = get_parameter("known_chain_frame").as_string();
  config.marker_frame = get_parameter("marker_frame").as_string();
  config.num_samples = static_cast<int>(get_parameter("num_samples").as_int());
  config.min_sample_interval_sec = get_parameter("min_sample_interval_sec").as_double();
  return config;
}

}  // namespace aruco_perception