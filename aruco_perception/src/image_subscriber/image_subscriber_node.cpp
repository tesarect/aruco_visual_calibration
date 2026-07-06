#include "aruco_perception/image_subscriber_node.hpp"

#include <cv_bridge/cv_bridge.h>

namespace aruco_perception
{

ImageSubscriberNode::ImageSubscriberNode()
: Node(
    "image_subscriber_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
  config_(loadConfigFromParams())
{
  image_sub_ = image_transport::create_subscription(
    this, config_.image_topic,
    std::bind(&ImageSubscriberNode::imageCallback, this, std::placeholders::_1),
    "raw");

  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    config_.camera_info_topic, rclcpp::SensorDataQoS(),
    std::bind(&ImageSubscriberNode::cameraInfoCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(), "image_subscriber_node ready (image: '%s', camera_info: '%s')",
    config_.image_topic.c_str(), config_.camera_info_topic.c_str());
}

void ImageSubscriberNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  const cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 5000,
    "Received frame: %dx%d, encoding '%s'",
    cv_ptr->image.cols, cv_ptr->image.rows, msg->encoding.c_str());
}

void ImageSubscriberNode::cameraInfoCallback(
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

ImageSubscriberConfig ImageSubscriberNode::loadConfigFromParams() const
{
  ImageSubscriberConfig config;
  config.image_topic = get_parameter("image_topic").as_string();
  config.camera_info_topic = get_parameter("camera_info_topic").as_string();
  return config;
}

}  // namespace aruco_perception