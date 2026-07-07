#ifndef ARUCO_PERCEPTION__ARUCO_DETECTOR_NODE_HPP_
#define ARUCO_PERCEPTION__ARUCO_DETECTOR_NODE_HPP_

#include <array>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/aruco.hpp>

namespace aruco_perception
{

/// Maps a dictionary name (e.g. "DICT_4X4_50" marker spec:
/// 4x4 dictionary, 50/100/250/1000 markers) to OpenCV's predefined
/// dictionary ID. The dictionary is the set of valid bit-patterns the
/// detector matches candidate squares against — not the marker's physical
/// size (see ArucoDetectorConfig::marker_length_m for that). Throws
/// std::invalid_argument for an unrecognized name.
cv::aruco::PREDEFINED_DICTIONARY_NAME dictionaryFromName(const std::string & name);

/// Tuning for ArucoDetectorNode, meant to be loaded from a parameter file.
/// Kept separate from code (rather than hardcoded) because real-world
/// lighting is inconsistent, unlike sim — these are exactly the knobs
/// expected to need retuning when moving from sim to the real robot.
struct ArucoDetectorConfig
{
  std::string image_topic;
  std::string camera_info_topic;
  /// Output topic for the detected marker's pose (camera optical frame).
  std::string pose_topic;
  /// When true, publish overlay_image_topic: the camera frame with the
  /// detected marker's XYZ axes drawn on it, for visual verification in
  /// rviz2/rqt_image_view. Off by default — this is a visualization aid,
  /// not something downstream nodes consume.
  bool publish_overlay_image = false;
  /// Output topic for the axis-overlay image (only used if
  /// publish_overlay_image is true).
  std::string overlay_image_topic;
  /// BGR color (OpenCV's channel order) for the marker's border, drawn by
  /// drawDetectedMarkers. Default is yellow.
  std::array<int, 3> overlay_border_color_bgr = {0, 255, 255};
  /// ArUco dictionary name, e.g. "DICT_4X4_50" (see CLAUDE.md marker specs).
  std::string dictionary_name;
  /// Only marker detections with this ID are published; others are ignored.
  /// Guards against stray/misread markers in the scene.
  int marker_id = 0;
  /// Physical marker side length in meters, used for pose scale in
  /// estimatePoseSingleMarkers. CLAUDE.md specifies 45mm diameter.
  double marker_length_m = 0.045;
  /// cv::aruco::DetectorParameters::adaptiveThreshWinSizeMin/Max/Step —
  /// window sizes tried for adaptive thresholding when binarizing the
  /// image before contour search. Wider range = more robust to uneven
  /// real-world lighting, at the cost of detection speed.
  int adaptive_thresh_win_size_min = 3;
  int adaptive_thresh_win_size_max = 23;
  int adaptive_thresh_win_size_step = 10;
  /// cv::aruco::DetectorParameters::adaptiveThreshConstant — constant
  /// subtracted from the mean during adaptive thresholding. Lower values
  /// pick up more/dimmer edges (useful in low light) but risk more noise.
  double adaptive_thresh_constant = 7.0;
  /// cv::aruco::DetectorParameters::minMarkerPerimeterRate — minimum
  /// candidate contour perimeter, as a fraction of the image's largest
  /// dimension. Filters out contours too small to plausibly be the marker.
  double min_marker_perimeter_rate = 0.03;
  /// cv::aruco::DetectorParameters::cornerRefinementMethod — 0=none,
  /// 1=subpixel, 2=contour, 3=AprilTag. Subpixel refinement improves pose
  /// accuracy at a small speed cost.
  int corner_refinement_method = 1;
};

/// Vision-only node: detects the single expected ArUco marker in the
/// camera feed and publishes its pose (camera optical frame -> marker) as
/// PoseStamped. Does not touch TF or robot frames — see
/// CalibrationBroadcasterNode for chaining this into camera->base_link.
///
/// Targets OpenCV 4.5.4's free-function ArUco API (as shipped with ROS 2
/// Humble's apt packages) rather than the newer ArucoDetector class
/// (OpenCV 4.7+), to avoid a second OpenCV build conflicting with
/// cv_bridge/image_transport's ABI.
class ArucoDetectorNode : public rclcpp::Node
{
public:
  ArucoDetectorNode();

private:
  ArucoDetectorConfig loadConfigFromParams() const;

  /// Builds a cv::aruco::DetectorParameters from config_'s tunables.
  cv::Ptr<cv::aruco::DetectorParameters> buildDetectorParams() const;

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);

  ArucoDetectorConfig config_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;

  image_transport::Subscriber image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  image_transport::Publisher overlay_image_pub_;

  /// Camera intrinsics, captured from camera_info on first receipt —
  /// required by estimatePoseSingleMarkers and assumed constant thereafter.
  cv::Mat camera_matrix_;
  cv::Mat distortion_coeffs_;
  bool camera_info_received_ = false;
};

}  // namespace aruco_perception

#endif  // ARUCO_PERCEPTION__ARUCO_DETECTOR_NODE_HPP_