#ifndef DEPTH_PERCEPTION__DEPTH_PERCEPTION_NODE_HPP_
#define DEPTH_PERCEPTION__DEPTH_PERCEPTION_NODE_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>

namespace depth_perception
{

/*
 * Per-environment (sim/real) topic names for DepthPerceptionNode, loaded
 * from a parameter file rather than hardcoded — sim's wrist-mounted RGBD
 * sensor and real's wall-mounted D415 (over Zenoh) publish under different
 * topic names, matching the sim/real config split used everywhere else in
 * this project (see aruco_perception's ImageSubscriberConfig).
 */
struct DepthPerceptionConfig
{
  // Topic carrying the color (RGB) image — used later to locate the
  // cupholder tray's rough position before searching the depth data.
  std::string rgb_image_topic;

  // Topic carrying the depth image (one distance-in-meters value per
  // pixel) — used later to find the tray's flat surface and the 4 holes
  // as gaps/dips in that surface.
  std::string depth_image_topic;

  // Topic carrying the sensor_msgs/CameraInfo (focal length, optical
  // center, distortion) needed to convert a 2D pixel + depth value into
  // an actual 3D point relative to the camera.
  std::string camera_info_topic;
};

/*
 * Step 1 plumbing-only node: subscribes to the RGB image, depth image, and
 * camera_info topics and logs that data is actually arriving, with basic
 * stats (resolution, encoding). No hole/cupholder detection logic yet —
 * this exists purely to confirm the camera inputs are readable before any
 * vision logic is added on top, matching aruco_perception's own
 * ImageSubscriberNode precedent (built as its own first checkpoint before
 * ArUco detection was added).
 */
class DepthPerceptionNode : public rclcpp::Node
{
public:
  DepthPerceptionNode();

private:
  // Reads all topic names from this node's declared parameters. Requires
  // the node to be started with a parameter file providing every field
  // (e.g. via automatically_declare_parameters_from_overrides).
  DepthPerceptionConfig loadConfigFromParams() const;

  // Logs the RGB frame's dimensions/encoding once per throttle period.
  void rgbImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  // Logs the depth frame's dimensions/encoding once per throttle period.
  // Depth images use a different encoding (e.g. 32FC1: one float, in
  // meters, per pixel) than the RGB image, so this is a separate callback
  // rather than reusing rgbImageCallback.
  void depthImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  // Logs that camera intrinsics were received — only once, since
  // camera_info is republished at a steady rate and doesn't change
  // between frames.
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);

  DepthPerceptionConfig config_;

  image_transport::Subscriber rgb_image_sub_;
  image_transport::Subscriber depth_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  bool camera_info_received_ = false;
};

}  // namespace depth_perception

#endif  // DEPTH_PERCEPTION__DEPTH_PERCEPTION_NODE_HPP_
