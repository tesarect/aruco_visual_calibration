#ifndef DEPTH_PERCEPTION__DEPTH_PERCEPTION_NODE_HPP_
#define DEPTH_PERCEPTION__DEPTH_PERCEPTION_NODE_HPP_

#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visual_calibration_msgs/msg/detection2_d_array.hpp>

namespace depth_perception
{

/*
 * STAGE 1 REBUILD (2026-08-05, branch tf-construction-rebuild) — see this
 * branch's own doc comment at the top of depth_perception_node.cpp for the
 * full rationale. This is a DELIBERATELY minimal rewrite, built up one
 * verifiable step at a time instead of porting the old file's full feature
 * set at once — the old file (preserved as depth_perception_node.
 * OLD_REFERENCE.{hpp,cpp}, excluded from the CMake build by not being
 * referenced there) had NO bug in its own arithmetic (confirmed via
 * extensive live numeric verification this session) but its final
 * broadcast TF's correctness depends entirely on calibration_broadcaster_
 * node's camera orientation, which could not be confidently root-caused
 * from log archaeology alone — see this branch's plan doc.
 *
 * Stage 1 scope: cup_holder ONLY (no holes yet, no TF broadcast yet, no
 * rolling-window filtering yet) — just get the 2D-pixel-to-camera-frame-3D
 * computation right, with EVERY intermediate quantity logged in one line,
 * so a future debugging session never again needs a custom Python capture
 * script (see resources/scripts/python/capture_tf_snapshot.py, written
 * this same session specifically because the old file's own logging
 * didn't expose enough to debug cleanly) to see what this node actually
 * computed at a given moment.
 */
struct DepthPerceptionConfig
{
  std::string rgb_image_topic;
  std::string depth_image_topic;
  std::string camera_info_topic;
  std::string detections_2d_topic;

  // Multiplies every raw depth-image value to convert it to meters — 1.0
  // for 32FC1 (already meters, sim's Gazebo plugin), 0.001 for 16UC1
  // (millimeters, real's D415). See depthImageCallback's own comment.
  double depth_scale_to_meters = 1.0;

  // Half-width (pixels) of the square patch sampled around the cup_holder
  // centroid before reducing to one depth value (median). e.g. 2 = 5x5
  // patch. Stage 1: cup_holder only, always median (no per-class branch
  // yet — that returns in a later stage alongside hole support).
  int depth_patch_half_size_px = 2;

  // --- Stage 2 additions (2026-08-05): camera-frame -> base_link TF ---
  // TF frame calibration_broadcaster_node's own known_chain_frame names —
  // MUST match calibration_broadcaster_{sim,real}.yaml's own value
  // exactly ("base_link" in both envs, confirmed).
  std::string known_chain_frame = "base_link";

  // Appended to the incoming detections_2d message's own header.frame_id
  // (the camera's own optical frame, e.g. "D415_color_optical_frame") to
  // form the calibrated camera frame to look up — e.g.
  // "D415_color_optical_frame_calibrated". MUST match
  // calibration_broadcaster_node's own broadcast_frame_suffix exactly.
  // Deliberately NOT hardcoding a per-env camera frame name here — the
  // incoming message's own frame_id already tells us which camera is in
  // play, sim or real.
  std::string broadcast_frame_suffix = "_calibrated";
};

/*
 * One instance's back-projection result, INCLUDING every intermediate
 * quantity that went into it — not just the final camera-frame X/Y/Z (the
 * old file's own BackProjectedPoint had position only, which is why this
 * session needed a from-scratch capture script to see the pixel/depth
 * inputs alongside the output). Logging this struct's own fields directly
 * means a single log line always carries everything needed to
 * independently recompute/verify the result by hand.
 */
struct CentroidBackProjection
{
  bool valid = false;

  // Inputs (2D)
  double cx_px = 0.0;
  double cy_px = 0.0;
  double depth_m = 0.0;  // reduced (median) depth over the sampled patch
  int patch_half_px = 0;
  int patch_valid_sample_count = 0;  // how many pixels in the patch had a real depth reading

  // Camera intrinsics actually used (so a log line is self-contained even
  // if intrinsics change between runs — no need to cross-reference a
  // separate camera_info capture).
  double fx = 0.0;
  double fy = 0.0;
  double cx_intrinsic = 0.0;
  double cy_intrinsic = 0.0;

  // Output (3D, camera's own optical frame: +X right, +Y down, +Z
  // forward — same convention as aruco_detector_node's marker_pose).
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

/*
 * Stage 2 (2026-08-05): the camera-frame -> base_link conversion result,
 * INCLUDING the exact calibrated camera TF used to produce it — same
 * "every intermediate quantity, not just the final number" philosophy as
 * CentroidBackProjection. A single log line built from this struct lets a
 * future debugging session verify the full chain (camera-frame point +
 * camera calibration -> base_link point) by hand, from ROS log output
 * alone, with no separate TF-echo capture needed.
 */
struct CameraToBaseLinkResult
{
  bool valid = false;
  std::string reason;  // set when !valid, e.g. "camera TF lookup failed: <exception message>"

  // The calibrated camera TF actually used (base_link -> calibrated
  // camera frame) — translation + a full RPY breakdown (not just the raw
  // quaternion) so a log line is directly human-readable without needing
  // to run the quaternion through a converter by hand.
  std::string calibrated_camera_frame;
  double cam_tx = 0.0, cam_ty = 0.0, cam_tz = 0.0;
  double cam_roll_deg = 0.0, cam_pitch_deg = 0.0, cam_yaw_deg = 0.0;

  // Final result: the centroid's position in known_chain_frame (base_link).
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

class DepthPerceptionNode : public rclcpp::Node
{
public:
  DepthPerceptionNode();

private:
  void rgbImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void depthImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);
  void detections2dCallback(
    const visual_calibration_msgs::msg::Detection2DArray::ConstSharedPtr & msg);

  // The one computation this stage exists to get right and make fully
  // observable — see CentroidBackProjection's own doc comment for why
  // every field of the result (not just x/y/z) is populated and logged.
  CentroidBackProjection backProjectCentroid(double cx_px, double cy_px) const;

  // Stage 2 (2026-08-05): looks up calibration_broadcaster_node's
  // broadcast camera TF (known_chain_frame -> header_frame_id +
  // broadcast_frame_suffix) and applies it to a camera-frame point,
  // returning EVERY intermediate quantity (see CameraToBaseLinkResult's
  // own doc comment) — not just the final base_link x/y/z.
  // header_frame_id is the incoming detection message's own camera
  // frame_id (e.g. "D415_color_optical_frame"), NOT hardcoded per-env.
  CameraToBaseLinkResult cameraFrameToBaseLink(
    double cam_x, double cam_y, double cam_z, const std::string & header_frame_id) const;

  DepthPerceptionConfig loadConfigFromParams() const;

  DepthPerceptionConfig config_;

  image_transport::Subscriber rgb_image_sub_;
  image_transport::Subscriber depth_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<visual_calibration_msgs::msg::Detection2DArray>::SharedPtr
    detections_2d_sub_;

  bool camera_info_received_ = false;
  double fx_ = 0.0;
  double fy_ = 0.0;
  double cx_intrinsic_ = 0.0;
  double cy_intrinsic_ = 0.0;

  bool depth_received_ = false;
  mutable std::mutex depth_mutex_;
  cv::Mat latest_depth_;  // CV_32FC1, meters, guarded by depth_mutex_

  // Stage 2 additions
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster instance_tf_broadcaster_;
};

}  // namespace depth_perception

#endif  // DEPTH_PERCEPTION__DEPTH_PERCEPTION_NODE_HPP_
