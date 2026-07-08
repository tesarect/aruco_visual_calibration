#ifndef ARUCO_PERCEPTION__CALIBRATION_BROADCASTER_NODE_HPP_
#define ARUCO_PERCEPTION__CALIBRATION_BROADCASTER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visual_calibration_msgs/action/calibrate.hpp>

#include "aruco_perception/orientation_averaging.hpp"

namespace aruco_perception
{

/// Tuning for CalibrationBroadcasterNode, loaded from a parameter file.
/// known_chain_frame/marker_frame name the TF chain we already know from
/// the robot's own kinematics (joint states) — which frame is "known" and
/// which is "the fixed unknown we're solving for" (the camera) depends on
/// the physical mounting: in sim the camera is wrist-mounted (marker and
/// camera both ride the arm, base_link->marker is known); on the real
/// robot the camera may instead be wall/ceiling-mounted (base_link->camera
/// is what's fixed and unknown, arm carries the marker) — see
/// progress.md's Open Verification Items. This node's logic is identical
/// either way; only these two param values change per environment.
struct CalibrationBroadcasterConfig
{
  /// Topic carrying the detector's camera_frame -> marker PoseStamped
  /// (see ArucoDetectorNode).
  std::string marker_pose_topic;
  /// TF frame at the base of the already-known chain (e.g. "base_link").
  std::string known_chain_frame;
  /// TF frame at the end of the already-known chain, matching the physical
  /// marker's mount (e.g. "rg2_gripper_aruco_link").
  std::string marker_frame;
  /// Number of samples to average before broadcasting.
  int num_samples = 10;
  /// Minimum time between accepted samples — lets you move the arm/marker
  /// between samples (e.g. via trajectory_planner's ~/trace_polygon) so
  /// consecutive samples aren't correlated frames of a stationary pose.
  double min_sample_interval_sec = 2.0;
  /// Priority for OrientationAveragingMethod::kSumNormalize; 0 disables it.
  /// See selectAveragingMethod — lower positive number = tried first.
  int orientation_sum_normalize_priority = 1;
  /// Priority for OrientationAveragingMethod::kMarkley; 0 disables it.
  /// NOT YET IMPLEMENTED — leave at 0 until it exists (see
  /// orientation_averaging.hpp), otherwise finishCalibration() throws if
  /// this method is actually selected.
  int orientation_markley_priority = 0;
};

/// Solves for the fixed transform between known_chain_frame and the
/// camera, from N samples of (known_chain_frame -> marker_frame, from TF)
/// chained with (camera -> marker_frame, from the detector's PoseStamped,
/// inverted), then broadcasts it once as a static TF. Collection starts
/// only when a ~/calibrate action goal is accepted — passive otherwise,
/// ignoring marker_pose_topic entirely. An action (not a plain service) is
/// used deliberately: calibration takes several seconds to minutes
/// (depending on how quickly samples accumulate), and a future web UI
/// needs live progress (samples_collected/samples_total feedback), not
/// just a final result — see Calibrate.action.
///
/// Position: arithmetic mean of all samples. Orientation: averaged via
/// whichever OrientationAveragingMethod selectAveragingMethod picks from
/// config_'s priorities (kSumNormalize today; kMarkley reserved for a more
/// robust average later — see orientation_averaging.hpp). Both the
/// resulting spread metrics are included in the action result and logged,
/// as a signal for whether the average is trustworthy — not yet used to
/// auto-escalate between methods (see progress.md's Feature Additions).
class CalibrationBroadcasterNode : public rclcpp::Node
{
public:
  using Calibrate = visual_calibration_msgs::action::Calibrate;
  using GoalHandleCalibrate = rclcpp_action::ServerGoalHandle<Calibrate>;

  CalibrationBroadcasterNode();

private:
  CalibrationBroadcasterConfig loadConfigFromParams() const;

  void markerPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr & msg);

  /// Accepts a new goal unless calibration is already in progress.
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Calibrate::Goal> goal);

  /// Always accepts cancellation requests.
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleCalibrate> goal_handle);

  /// Stores goal_handle_ and resets collection state — the actual sample
  /// collection happens in markerPoseCallback as marker_pose messages
  /// arrive, not in a dedicated execution thread (no blocking work here).
  void handleAccepted(const std::shared_ptr<GoalHandleCalibrate> goal_handle);

  /// Averages collected_positions_ (arithmetic mean) and
  /// collected_orientations_ (via averaging_method_), broadcasts
  /// known_chain_frame -> the camera frame (from the most recent sample's
  /// header.frame_id) as a static TF, and completes goal_handle_ with the
  /// result (see Calibrate.action). Logs the orientation spread metrics.
  /// Clears both collected_ vectors and resets is_collecting_.
  void finishCalibration();

  CalibrationBroadcasterConfig config_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;
  /// Selected once at construction from config_'s priorities — see
  /// selectAveragingMethod.
  OrientationAveragingMethod averaging_method_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr marker_pose_sub_;
  rclcpp_action::Server<Calibrate>::SharedPtr calibrate_action_server_;

  bool is_collecting_ = false;
  /// The in-progress goal's handle — used to publish feedback per sample
  /// and to complete with a result in finishCalibration(). Null when not
  /// collecting.
  std::shared_ptr<GoalHandleCalibrate> goal_handle_;
  rclcpp::Time last_sample_time_;
  std::vector<geometry_msgs::msg::Vector3> collected_positions_;
  std::vector<tf2::Quaternion> collected_orientations_;
  /// The most recent sample's camera frame_id — carried through to the
  /// final broadcast's child_frame_id.
  geometry_msgs::msg::PoseStamped last_sample_;
};

}  // namespace aruco_perception

#endif  // ARUCO_PERCEPTION__CALIBRATION_BROADCASTER_NODE_HPP_
