#include "depth_perception/depth_perception_node.hpp"

#include <cv_bridge/cv_bridge.h>

namespace depth_perception
{

DepthPerceptionNode::DepthPerceptionNode()
: Node(
    "depth_perception_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
  config_(loadConfigFromParams())
{
  rgb_image_sub_ = image_transport::create_subscription(
    this, config_.rgb_image_topic,
    std::bind(&DepthPerceptionNode::rgbImageCallback, this, std::placeholders::_1),
    "raw");

  depth_image_sub_ = image_transport::create_subscription(
    this, config_.depth_image_topic,
    std::bind(&DepthPerceptionNode::depthImageCallback, this, std::placeholders::_1),
    "raw");

  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    config_.camera_info_topic, rclcpp::SensorDataQoS(),
    std::bind(&DepthPerceptionNode::cameraInfoCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "depth_perception_node ready (rgb: '%s', depth: '%s', camera_info: '%s')",
    config_.rgb_image_topic.c_str(), config_.depth_image_topic.c_str(),
    config_.camera_info_topic.c_str());
}

void DepthPerceptionNode::rgbImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  const cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 5000,
    "Received RGB frame: %dx%d, encoding '%s'",
    cv_ptr->image.cols, cv_ptr->image.rows, msg->encoding.c_str());
}

void DepthPerceptionNode::depthImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  // toCvShare (not toCvCopy) is safe here: we're only reading dimensions
  // and encoding, no pixel-format conversion is requested, so no actual
  // copy is needed yet. See aruco_perception's error-mitigation notes on
  // toCvShare vs toCvCopy — toCvShare only breaks when a real encoding
  // conversion is asked of it, which this callback deliberately does not do.
  const cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 5000,
    "Received depth frame: %dx%d, encoding '%s'",
    cv_ptr->image.cols, cv_ptr->image.rows, msg->encoding.c_str());
}

void DepthPerceptionNode::cameraInfoCallback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
{
  if (camera_info_received_) {
    return;
  }
  camera_info_received_ = true;

  RCLCPP_INFO(
    get_logger(), "Received camera_info: %ux%u, distortion model '%s'",
    msg->width, msg->height, msg->distortion_model.c_str());
}

DepthPerceptionConfig DepthPerceptionNode::loadConfigFromParams() const
{
  DepthPerceptionConfig config;
  config.rgb_image_topic = get_parameter("rgb_image_topic").as_string();
  config.depth_image_topic = get_parameter("depth_image_topic").as_string();
  config.camera_info_topic = get_parameter("camera_info_topic").as_string();
  return config;
}

}  // namespace depth_perception
