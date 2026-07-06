#ifndef ARUCO_PERCEPTION__IMAGE_SUBSCRIBER_NODE_HPP_
#define ARUCO_PERCEPTION__IMAGE_SUBSCRIBER_NODE_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>

namespace aruco_perception
{

/// Per-environment (sim/real) topic names for ImageSubscriberNode, meant to
/// be loaded from a parameter file — sim and real publish the camera feed
/// under different topic names (real robot arrives over Zenoh, see
/// CLAUDE.md), so this is a parameter rather than a hardcoded string.
struct ImageSubscriberConfig
{
  /// Topic to subscribe to via image_transport (raw or compressed).
  std::string image_topic;
  /// Topic carrying the matching sensor_msgs/CameraInfo (intrinsics).
  std::string camera_info_topic;
};

/// Plumbing-only node: subscribes to the camera's image and camera_info
/// topics and logs that data is arriving. No ArUco detection yet — this
/// exists to verify the topics/conversion work before adding vision logic.
class ImageSubscriberNode : public rclcpp::Node
{
public:
  ImageSubscriberNode();

private:
  /// Reads image_topic and camera_info_topic from this node's declared
  /// parameters. Requires the node to have been started with a parameter
  /// file providing both (e.g. via automatically_declare_parameters_from_overrides).
  ImageSubscriberConfig loadConfigFromParams() const;

  /// Logs image dimensions/encoding once per throttle period, and converts
  /// via cv_bridge to confirm the ROS Image -> cv::Mat path works.
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  /// Logs that camera intrinsics were received — only once, since
  /// camera_info is latched/republished at a steady rate and doesn't
  /// change between frames.
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);

  ImageSubscriberConfig config_;
  image_transport::Subscriber image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  bool camera_info_received_ = false;
};

}  // namespace aruco_perception

#endif  // ARUCO_PERCEPTION__IMAGE_SUBSCRIBER_NODE_HPP_