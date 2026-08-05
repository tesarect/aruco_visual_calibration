// STAGE 1 REBUILD (2026-08-05, branch tf-construction-rebuild).
//
// Why this file was rewritten from scratch instead of patched: a long
// investigation session traced a real-robot symptom (cup_holder/hole TFs
// visually displaced/mirrored in RViz relative to their true physical
// positions) down to the old depth_perception_node.cpp's math — and found
// NO bug there. Every formula was reproduced by hand against live
// tf2_echo/log captures and matched exactly. The actual open question
// (whether calibration_broadcaster_node's camera ORIENTATION result is
// itself correct) could not be conclusively resolved from log archaeology
// alone, because the old file's own logging only ever exposed FINAL
// numbers (e.g. "cup_holder: frame(x=.., y=.., z=..)"), not the
// intermediate pixel/depth/intrinsics inputs that produced them — so
// verifying anything required either re-deriving inputs by inverting the
// output formula (error-prone, confirmed to produce a physically
// impossible result once this session) or writing an ad-hoc Python
// capture script (resources/scripts/python/capture_tf_snapshot.py) just
// to get one clean, single-moment snapshot.
//
// This rebuild's actual GOAL is not different math — it's making every
// step of the computation independently observable from the node's own
// log output, one small, testable stage at a time, so a future debugging
// session never needs a special script to see what this node is actually
// doing. See CentroidBackProjection's own doc comment (in the header) for
// what "every step" means concretely.
//
// SCOPE OF THIS STAGE: cup_holder centroid only. No holes yet, no TF
// broadcast yet, no rolling-window/stability filtering yet, no overlay
// image yet. The old, full-featured file is preserved as
// depth_perception_node.OLD_REFERENCE.{hpp,cpp} (NOT referenced by
// CMakeLists.txt, so it does not build) purely as a reference for porting
// the remaining features back in, stage by stage, once this stage is
// confirmed correct on real hardware.

#include "depth_perception/depth_perception_node.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

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

  detections_2d_sub_ = create_subscription<visual_calibration_msgs::msg::Detection2DArray>(
    config_.detections_2d_topic, rclcpp::QoS(10),
    std::bind(&DepthPerceptionNode::detections2dCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "depth_perception_node ready [STAGE 1 REBUILD, cup_holder-only] (rgb: '%s', depth: '%s', "
    "camera_info: '%s', detections_2d: '%s', depth_patch_half_size_px: %d)",
    config_.rgb_image_topic.c_str(), config_.depth_image_topic.c_str(),
    config_.camera_info_topic.c_str(), config_.detections_2d_topic.c_str(),
    config_.depth_patch_half_size_px);
}

void DepthPerceptionNode::rgbImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  // Subscribed only so this node's log shows the color feed is alive —
  // not used for any math (2D detection already ran upstream, in
  // yolo_marker_bridge_node/cup_holder_detector_node).
  RCLCPP_INFO_ONCE(
    get_logger(), "Receiving RGB frames on '%s' (%dx%d, encoding '%s').",
    config_.rgb_image_topic.c_str(), msg->width, msg->height, msg->encoding.c_str());
}

void DepthPerceptionNode::depthImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  // toCvCopy (not toCvShare): converts to a fixed CV_32FC1 representation
  // regardless of incoming encoding (32FC1 sim vs 16UC1 real) — toCvShare
  // cannot perform a real pixel-format conversion.
  const cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);

  RCLCPP_INFO_ONCE(
    get_logger(), "Receiving depth frames on '%s' (%dx%d, encoding '%s', scale_to_meters=%.4f).",
    config_.depth_image_topic.c_str(), msg->width, msg->height, msg->encoding.c_str(),
    config_.depth_scale_to_meters);

  cv::Mat depth_meters;
  if (msg->encoding == "32FC1") {
    depth_meters = cv_ptr->image * config_.depth_scale_to_meters;
  } else {
    // 16UC1 (millimeters) is real's own encoding — convert to float first
    // so the multiply doesn't saturate/truncate as an integer type.
    cv_ptr->image.convertTo(depth_meters, CV_32F, config_.depth_scale_to_meters);
  }

  std::lock_guard<std::mutex> lock(depth_mutex_);
  latest_depth_ = depth_meters;
  depth_received_ = true;
}

void DepthPerceptionNode::cameraInfoCallback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
{
  if (camera_info_received_) {
    return;
  }
  // msg->k is row-major 3x3: [fx 0 cx; 0 fy cy; 0 0 1].
  fx_ = msg->k[0];
  fy_ = msg->k[4];
  cx_intrinsic_ = msg->k[2];
  cy_intrinsic_ = msg->k[5];
  camera_info_received_ = true;

  RCLCPP_INFO(
    get_logger(), "Received camera_info: %ux%u, fx=%.4f fy=%.4f cx=%.4f cy=%.4f",
    msg->width, msg->height, fx_, fy_, cx_intrinsic_, cy_intrinsic_);
}

CentroidBackProjection DepthPerceptionNode::backProjectCentroid(double cx_px, double cy_px) const
{
  CentroidBackProjection result;
  result.cx_px = cx_px;
  result.cy_px = cy_px;
  result.patch_half_px = config_.depth_patch_half_size_px;
  result.fx = fx_;
  result.fy = fy_;
  result.cx_intrinsic = cx_intrinsic_;
  result.cy_intrinsic = cy_intrinsic_;

  std::lock_guard<std::mutex> lock(depth_mutex_);
  if (latest_depth_.empty()) {
    return result;
  }

  const int u = static_cast<int>(std::lround(cx_px));
  const int v = static_cast<int>(std::lround(cy_px));
  const int half = config_.depth_patch_half_size_px;

  const int u0 = std::max(0, u - half);
  const int v0 = std::max(0, v - half);
  const int u1 = std::min(latest_depth_.cols - 1, u + half);
  const int v1 = std::min(latest_depth_.rows - 1, v + half);

  if (u0 > u1 || v0 > v1) {
    // Centroid pixel falls entirely outside the depth image (e.g.
    // mismatched RGB/depth resolution) — nothing to sample.
    return result;
  }

  std::vector<float> samples;
  samples.reserve(static_cast<size_t>((u1 - u0 + 1) * (v1 - v0 + 1)));
  for (int v = v0; v <= v1; ++v) {
    for (int u2 = u0; u2 <= u1; ++u2) {
      const float depth = latest_depth_.at<float>(v, u2);
      // Zero/NaN is the depth camera's "no return at this pixel" signal
      // (common right at a cavity's rim) — not a real distance of zero.
      if (std::isfinite(depth) && depth > 0.0f) {
        samples.push_back(depth);
      }
    }
  }
  result.patch_valid_sample_count = static_cast<int>(samples.size());

  if (samples.empty()) {
    return result;
  }

  // Median — robust to a minority of outlier reads in the patch.
  std::sort(samples.begin(), samples.end());
  result.depth_m = samples[samples.size() / 2];

  // Standard pinhole back-projection. cx_px/cy_px are PIXEL coordinates
  // (cx_px grows rightward, cy_px grows downward, standard image
  // convention) — this formula defines the camera-frame X/Y DIRECTLY from
  // those same pixel axes (image-right -> camera +X/right, image-down ->
  // camera +Y/down): there is no separate "3D pose" axis convention this
  // needs to be reconciled against, by construction.
  result.x = (cx_px - cx_intrinsic_) * result.depth_m / fx_;
  result.y = (cy_px - cy_intrinsic_) * result.depth_m / fy_;
  result.z = result.depth_m;
  result.valid = true;
  return result;
}

void DepthPerceptionNode::detections2dCallback(
  const visual_calibration_msgs::msg::Detection2DArray::ConstSharedPtr & msg)
{
  if (!camera_info_received_ || !depth_received_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Got detections_2d but still waiting on camera_info/depth before back-projecting.");
    return;
  }

  for (const auto & detection : msg->detections) {
    // Stage 1: cup_holder only. Holes/aruco_marker are skipped entirely
    // for now — see this file's own top-of-file doc comment for the
    // staged-rebuild plan.
    if (detection.class_name != "cup_holder") {
      continue;
    }

    const CentroidBackProjection result = backProjectCentroid(detection.cx, detection.cy);

    if (result.valid) {
      // Every intermediate quantity in ONE line — the whole point of this
      // rebuild. A future debugging session can verify this by hand from
      // this single line alone: no separate camera_info capture, no
      // separate patch-sampling capture, nothing implicit.
      RCLCPP_INFO(
        get_logger(),
        "cup_holder centroid: pixel(cx=%.2f, cy=%.2f) patch_half_px=%d "
        "valid_samples=%d/%d depth_m=%.4f intrinsics(fx=%.4f, fy=%.4f, cx=%.4f, cy=%.4f) "
        "-> camera_frame(x=%.4f, y=%.4f, z=%.4f)",
        result.cx_px, result.cy_px, result.patch_half_px, result.patch_valid_sample_count,
        (2 * result.patch_half_px + 1) * (2 * result.patch_half_px + 1), result.depth_m,
        result.fx, result.fy, result.cx_intrinsic, result.cy_intrinsic, result.x, result.y,
        result.z);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "cup_holder centroid: pixel(cx=%.2f, cy=%.2f) patch_half_px=%d valid_samples=%d/%d "
        "-> NO VALID DEPTH in the sampled patch",
        detection.cx, detection.cy, result.patch_half_px, result.patch_valid_sample_count,
        (2 * result.patch_half_px + 1) * (2 * result.patch_half_px + 1));
    }
  }
}

DepthPerceptionConfig DepthPerceptionNode::loadConfigFromParams() const
{
  DepthPerceptionConfig config;
  config.rgb_image_topic = get_parameter("rgb_image_topic").as_string();
  config.depth_image_topic = get_parameter("depth_image_topic").as_string();
  config.camera_info_topic = get_parameter("camera_info_topic").as_string();
  config.detections_2d_topic = get_parameter("detections_2d_topic").as_string();
  config.depth_scale_to_meters = get_parameter("depth_scale_to_meters").as_double();
  config.depth_patch_half_size_px =
    static_cast<int>(get_parameter("depth_patch_half_size_px").as_int());
  return config;
}

}  // namespace depth_perception
