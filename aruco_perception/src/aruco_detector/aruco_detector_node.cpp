#include "aruco_perception/aruco_detector_node.hpp"

#include <algorithm>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace aruco_perception
{

cv::aruco::PREDEFINED_DICTIONARY_NAME dictionaryFromName(const std::string & name)
{
  if (name == "DICT_4X4_50") {return cv::aruco::DICT_4X4_50;}
  if (name == "DICT_4X4_100") {return cv::aruco::DICT_4X4_100;}
  if (name == "DICT_4X4_250") {return cv::aruco::DICT_4X4_250;}
  if (name == "DICT_4X4_1000") {return cv::aruco::DICT_4X4_1000;}
  throw std::invalid_argument("Unknown ArUco dictionary_name: " + name);
}

ArucoDetectorNode::ArucoDetectorNode()
: Node(
    "aruco_detector_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
  config_(loadConfigFromParams()),
  dictionary_(cv::aruco::getPredefinedDictionary(dictionaryFromName(config_.dictionary_name))),
  detector_params_(buildDetectorParams())
{
  image_sub_ = image_transport::create_subscription(
    this, config_.image_topic,
    std::bind(&ArucoDetectorNode::imageCallback, this, std::placeholders::_1),
    "raw");

  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    config_.camera_info_topic, rclcpp::SensorDataQoS(),
    std::bind(&ArucoDetectorNode::cameraInfoCallback, this, std::placeholders::_1));

  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(config_.pose_topic, 10);

  if (config_.publish_overlay_image) {
    overlay_image_pub_ = image_transport::create_publisher(this, config_.overlay_image_topic);
  }

  RCLCPP_INFO(
    get_logger(), "aruco_detector_node ready (marker_id: %d, dictionary: '%s')",
    config_.marker_id, config_.dictionary_name.c_str());
}

cv::Ptr<cv::aruco::DetectorParameters> ArucoDetectorNode::buildDetectorParams() const
{
  cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
  params->adaptiveThreshWinSizeMin = config_.adaptive_thresh_win_size_min;
  params->adaptiveThreshWinSizeMax = config_.adaptive_thresh_win_size_max;
  params->adaptiveThreshWinSizeStep = config_.adaptive_thresh_win_size_step;
  params->adaptiveThreshConstant = config_.adaptive_thresh_constant;
  params->minMarkerPerimeterRate = config_.min_marker_perimeter_rate;
  params->cornerRefinementMethod = config_.corner_refinement_method;
  return params;
}

void ArucoDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  if (!camera_info_received_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "No camera_info received yet on '%s' — skipping detection (need intrinsics for pose).",
      config_.camera_info_topic.c_str());
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    // toCvCopy (not toCvShare) is required here: toCvShare only works when
    // no actual pixel conversion is needed, but sim publishes rgb8 while
    // ArUco detection needs mono8 — that conversion requires a copy.
    cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "cv_bridge conversion failed: %s", e.what());
    return;
  }

  // camera → marker frame
  std::vector<int> marker_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners;
  cv::aruco::detectMarkers(
    cv_ptr->image, dictionary_, marker_corners, marker_ids, detector_params_);

  const auto found = std::find(marker_ids.begin(), marker_ids.end(), config_.marker_id);
  if (found == marker_ids.end()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Marker id %d not detected in this frame (%zu other marker(s) seen).",
      config_.marker_id, marker_ids.size());
    return;
  }
  const size_t idx = static_cast<size_t>(std::distance(marker_ids.begin(), found));

  std::vector<std::vector<cv::Point2f>> single_marker_corners{marker_corners[idx]};
  std::vector<cv::Vec3d> rvecs, tvecs;
  cv::aruco::estimatePoseSingleMarkers(
    single_marker_corners, config_.marker_length_m, camera_matrix_, distortion_coeffs_,
    rvecs, tvecs);

  cv::Mat rotation_matrix;
  cv::Rodrigues(rvecs[0], rotation_matrix);
  tf2::Matrix3x3 tf_rotation(
    rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
    rotation_matrix.at<double>(0, 2),
    rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1),
    rotation_matrix.at<double>(1, 2),
    rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
    rotation_matrix.at<double>(2, 2));
  tf2::Quaternion tf_quaternion;
  tf_rotation.getRotation(tf_quaternion);

  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header = msg->header;
  pose_msg.pose.position.x = tvecs[0][0];
  pose_msg.pose.position.y = tvecs[0][1];
  pose_msg.pose.position.z = tvecs[0][2];
  pose_msg.pose.orientation.x = tf_quaternion.x();
  pose_msg.pose.orientation.y = tf_quaternion.y();
  pose_msg.pose.orientation.z = tf_quaternion.z();
  pose_msg.pose.orientation.w = tf_quaternion.w();
  pose_pub_->publish(pose_msg);

  if (config_.publish_overlay_image) {
    // Separate bgr8 conversion (rather than reusing cv_ptr's mono8 buffer):
    // drawAxis's red/green/blue lines and the yellow border are only
    // distinguishable on a color image. Skipped entirely when the feature
    // is off, so it costs nothing in the common (overlay-disabled) case.
    cv_bridge::CvImagePtr overlay_ptr;
    try {
      overlay_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "cv_bridge overlay conversion failed: %s", e.what());
      return;
    }

    const cv::Scalar border_color(
      config_.overlay_border_color_bgr[0], config_.overlay_border_color_bgr[1],
      config_.overlay_border_color_bgr[2]);
    cv::aruco::drawDetectedMarkers(
      overlay_ptr->image, single_marker_corners, cv::noArray(), border_color);
    cv::aruco::drawAxis(
      overlay_ptr->image, camera_matrix_, distortion_coeffs_,
      rvecs[0], tvecs[0], config_.marker_length_m * 0.5f);

    overlay_image_pub_.publish(overlay_ptr->toImageMsg());
  }
}

void ArucoDetectorNode::cameraInfoCallback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
{
  if (camera_info_received_) {
    return;
  }

  camera_matrix_ = (cv::Mat_<double>(3, 3) <<
    msg->k[0], msg->k[1], msg->k[2],
    msg->k[3], msg->k[4], msg->k[5],
    msg->k[6], msg->k[7], msg->k[8]);
  distortion_coeffs_ = cv::Mat(msg->d.size(), 1, CV_64F);
  for (size_t i = 0; i < msg->d.size(); ++i) {
    distortion_coeffs_.at<double>(static_cast<int>(i)) = msg->d[i];
  }

  camera_info_received_ = true;
  RCLCPP_INFO(get_logger(), "Camera intrinsics captured from '%s'.",
    config_.camera_info_topic.c_str());
}

ArucoDetectorConfig ArucoDetectorNode::loadConfigFromParams() const
{
  ArucoDetectorConfig config;
  config.image_topic = get_parameter("image_topic").as_string();
  config.camera_info_topic = get_parameter("camera_info_topic").as_string();
  config.pose_topic = get_parameter("pose_topic").as_string();
  config.publish_overlay_image = get_parameter("publish_overlay_image").as_bool();
  config.overlay_image_topic = get_parameter("overlay_image_topic").as_string();

  const std::vector<int64_t> border_color_bgr =
    get_parameter("overlay_border_color_bgr").as_integer_array();
  config.overlay_border_color_bgr = {
    static_cast<int>(border_color_bgr[0]), static_cast<int>(border_color_bgr[1]),
    static_cast<int>(border_color_bgr[2])};

  config.dictionary_name = get_parameter("dictionary_name").as_string();
  config.marker_id = static_cast<int>(get_parameter("marker_id").as_int());
  config.marker_length_m = get_parameter("marker_length_m").as_double();
  config.adaptive_thresh_win_size_min =
    static_cast<int>(get_parameter("adaptive_thresh_win_size_min").as_int());
  config.adaptive_thresh_win_size_max =
    static_cast<int>(get_parameter("adaptive_thresh_win_size_max").as_int());
  config.adaptive_thresh_win_size_step =
    static_cast<int>(get_parameter("adaptive_thresh_win_size_step").as_int());
  config.adaptive_thresh_constant = get_parameter("adaptive_thresh_constant").as_double();
  config.min_marker_perimeter_rate = get_parameter("min_marker_perimeter_rate").as_double();
  config.corner_refinement_method =
    static_cast<int>(get_parameter("corner_refinement_method").as_int());
  return config;
}

}  // namespace aruco_perception