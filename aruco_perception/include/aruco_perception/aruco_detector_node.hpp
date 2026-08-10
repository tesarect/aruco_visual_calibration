#ifndef ARUCO_PERCEPTION__ARUCO_DETECTOR_NODE_HPP_
#define ARUCO_PERCEPTION__ARUCO_DETECTOR_NODE_HPP_

#include <array>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/aruco.hpp>
#include <visual_calibration_msgs/msg/detection2_d_array.hpp>

namespace aruco_perception
{

/// Maps a dictionary name (e.g. "DICT_4X4_50": 4x4 dictionary, 50/100/250/1000
/// markers) to OpenCV's predefined dictionary ID. The dictionary is the set of
/// valid bit-patterns the detector matches candidate squares against, not the
/// marker's physical size (see ArucoDetectorConfig::marker_length_m for that).
/// Throws std::invalid_argument for an unrecognized name.
cv::aruco::PREDEFINED_DICTIONARY_NAME dictionaryFromName(const std::string & name);

/// Tuning for ArucoDetectorNode, loaded from a parameter file rather than
/// hardcoded so detection thresholds can be retuned per lighting condition
/// (sim vs. real robot) without a rebuild.
struct ArucoDetectorConfig
{
  std::string image_topic;
  std::string camera_info_topic;
  /// Output topic for the detected marker's pose (camera optical frame).
  std::string pose_topic;
  /// When true, publish overlay_image_topic: the camera frame with the
  /// detected marker's XYZ axes drawn on it, for visual verification in
  /// rviz2/rqt_image_view. This is a visualization aid, not consumed by
  /// downstream nodes.
  bool publish_overlay_image = false;
  /// Output topic for the axis-overlay image (only used if
  /// publish_overlay_image is true).
  std::string overlay_image_topic;
  /// Topic for this node's ArUco pixel-space detections
  /// (visual_calibration_msgs/Detection2DArray, class_name "aruco_marker").
  /// Shared with yolo_marker_bridge_node.py, which publishes cup_holder/hole
  /// detections on the same topic, so downstream consumers (e.g.
  /// calibration_orchestrator_node's image-based centering) can read one
  /// unified detection stream regardless of which detector is active.
  std::string detections_2d_topic;
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
  /// cv::aruco::DetectorParameters::cornerRefinementWinSize — pixel-radius
  /// neighborhood searched around each initial corner guess during
  /// subpixel/contour refinement. OpenCV default 5.
  int corner_refinement_win_size = 5;
  /// cv::aruco::DetectorParameters::cornerRefinementMaxIterations —
  /// refinement's stop criteria: max iterations before giving up. OpenCV
  /// default 30.
  int corner_refinement_max_iterations = 30;
  /// cv::aruco::DetectorParameters::cornerRefinementMinAccuracy —
  /// refinement's stop criteria: minimum error to consider converged.
  /// OpenCV default 0.1.
  double corner_refinement_min_accuracy = 0.1;
  /// cv::aruco::DetectorParameters::polygonalApproxAccuracyRate — how
  /// loosely a candidate contour is approximated as a quadrilateral before
  /// its 4 corners are extracted, upstream of refinement. OpenCV default
  /// 0.03; too loose can visibly pull a corner in/out of an otherwise-square
  /// marker.
  double polygonal_approx_accuracy_rate = 0.03;
  /// Startup default for the "active" parameter (see class doc comment).
  /// Read once at construction; the live value is always re-read via
  /// get_parameter("active") in imageCallback (never cached), so a runtime
  /// set_parameters call takes effect on the next frame with no restart.
  bool active = true;
};

/// Vision-only node: detects the single expected ArUco marker in the camera
/// feed and publishes its pose (camera optical frame -> marker) as
/// PoseStamped. Does not touch TF or robot frames — see
/// CalibrationBroadcasterNode for chaining this into camera->base_link.
///
/// Targets OpenCV 4.5.4's free-function ArUco API (as shipped with ROS 2
/// Humble's apt packages) rather than the newer ArucoDetector class
/// (OpenCV 4.7+), to avoid a second OpenCV build conflicting with
/// cv_bridge/image_transport's ABI.
///
/// Classical/hybrid switch: this node and aruco_perception_yolo_bridge's
/// YoloMarkerBridgeNode both publish geometry_msgs/PoseStamped on the same
/// pose_topic (/aruco_perception/marker_pose by convention). Exactly one of
/// them is "active" at a time, gated by the "active" bool parameter (see
/// ArucoDetectorConfig::active). calibration_orchestrator_node flips exactly
/// one of the two nodes' "active" params true (and the other false) via the
/// standard ROS set_parameters service — no lifecycle nodes, no process
/// start/stop; both nodes stay running and subscribed the whole time, and
/// "active" just gates whether imageCallback does any real work this frame.
/// When inactive, imageCallback returns immediately, before running
/// detection at all, so the inactive detector spends no CPU on frames
/// nobody will use.
///
/// Overlay stream: overlay_image_topic is published on every processed
/// frame, not only frames where the marker was found, so a subscriber sees
/// a continuously live stream rather than a frozen last-good frame. Also
/// draws a crosshair at the image's pixel center whenever
/// get_parameter("show_centering_crosshair") is true, set by
/// calibration_orchestrator_node for the duration of its image-based
/// centering routine (centerOnMarkerUsingImage); same live-re-read pattern
/// as "active", so no restart is needed to toggle it.
///
/// Marker pixel data: when the marker is found, this node also publishes
/// its pixel-space centroid/bbox as a visual_calibration_msgs/Detection2D
/// (class_name "aruco_marker") on detections_2d_topic, the same topic
/// yolo_marker_bridge_node.py publishes cup_holder/hole detections on. This
/// lets calibration_orchestrator_node convert the marker's pixel offset
/// from image-center into a real-world correction without duplicating the
/// corner-detection math already done here by cv::aruco::detectMarkers.
class ArucoDetectorNode : public rclcpp::Node
{
public:
  ArucoDetectorNode();

private:
  ArucoDetectorConfig loadConfigFromParams() const;

  /// Builds a cv::aruco::DetectorParameters from config_'s tunables.
  cv::Ptr<cv::aruco::DetectorParameters> buildDetectorParams() const;

  /// Runs detection on each incoming frame, publishes pose/overlay/pixel
  /// detections, and logs marker visibility state transitions plus a
  /// throttled per-frame corner-squareness diagnostic (side/diagonal
  /// lengths of the detected quadrilateral) whenever the marker is found —
  /// useful for tuning the corner_refinement_*/polygonal_approx_accuracy_rate
  /// fields on ArucoDetectorConfig.
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);

  ArucoDetectorConfig config_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;

  image_transport::Subscriber image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  image_transport::Publisher overlay_image_pub_;
  /// See ArucoDetectorConfig::detections_2d_topic — published
  /// unconditionally, independent of the publish_overlay_image gate.
  rclcpp::Publisher<visual_calibration_msgs::msg::Detection2DArray>::SharedPtr detections_2d_pub_;

  /// Camera intrinsics, captured from camera_info on first receipt —
  /// required by estimatePoseSingleMarkers and assumed constant thereafter.
  cv::Mat camera_matrix_;
  cv::Mat distortion_coeffs_;
  bool camera_info_received_ = false;
  /// Image dimensions, captured alongside camera_matrix_/distortion_coeffs_
  /// in cameraInfoCallback — needed for the centering crosshair and (by
  /// calibration_orchestrator_node, which reads CameraInfo itself) the
  /// pixel-to-metric offset conversion.
  int image_width_ = 0;
  int image_height_ = 0;

  /// Tracks marker visibility as of the previous imageCallback so that a
  /// found/not-found transition is logged once (on change) rather than
  /// every frame. std::nullopt at startup so the first frame doesn't log a
  /// spurious "reappeared" transition.
  std::optional<bool> marker_was_visible_;
};

}  // namespace aruco_perception

#endif  // ARUCO_PERCEPTION__ARUCO_DETECTOR_NODE_HPP_
