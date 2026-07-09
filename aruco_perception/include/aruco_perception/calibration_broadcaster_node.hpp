#ifndef ARUCO_PERCEPTION__CALIBRATION_BROADCASTER_NODE_HPP_
#define ARUCO_PERCEPTION__CALIBRATION_BROADCASTER_NODE_HPP_

#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
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
#include <visual_calibration_msgs/srv/get_polygon_waypoints.hpp>
#include <visual_calibration_msgs/srv/trace_path.hpp>

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
  /// Appended to the detector's camera frame_id to form the broadcast TF's
  /// child_frame_id (e.g. "wrist_rgbd_camera_depth_optical_frame" becomes
  /// "..._calibrated"). Required: broadcasting under the exact same name
  /// as an existing URDF-declared frame would conflict with it in the TF
  /// tree (two disagreeing publishers for one frame) — this keeps our
  /// computed result distinct from any physically-declared camera frame,
  /// in both sim (where the URDF frame is sim's ground truth) and real
  /// (where it just avoids colliding with whatever frame name the real
  /// camera driver publishes, if any).
  std::string broadcast_frame_suffix = "_calibrated";
  /// Number of samples to average before broadcasting — one sample is
  /// taken per waypoint visited, so this many waypoints get requested
  /// from trajectory_planner (wrapping around its polygon if num_samples
  /// exceeds the polygon's corner count).
  int num_samples = 10;
  /// How long to wait for a fresh marker_pose message (published after
  /// the arm is confirmed settled at a waypoint — see
  /// requestSampleAfterSettling) before giving up on that sample and
  /// aborting the calibration run.
  double sample_wait_timeout_sec = 5.0;
  /// Planning mode requested on each ~/trace_path call — see
  /// TracePath::Request::PLANNING_MODE_*.
  uint8_t planning_mode =
    visual_calibration_msgs::srv::TracePath::Request::PLANNING_MODE_CARTESIAN;
  /// Priority for OrientationAveragingMethod::kSumNormalize; 0 disables it.
  /// See selectAveragingMethod — lower positive number = tried first.
  int orientation_sum_normalize_priority = 1;
  /// Priority for OrientationAveragingMethod::kMarkley; 0 disables it.
  /// NOT YET IMPLEMENTED — leave at 0 until it exists (see
  /// orientation_averaging.hpp), otherwise finishCalibration() throws if
  /// this method is actually selected.
  int orientation_markley_priority = 0;
};

/// Orchestrates calibration: fetches waypoints from trajectory_planner
/// (~/get_polygon_waypoints, read-only), then for each one — calls
/// trajectory_planner's ~/trace_path with just that single waypoint
/// (blocking until the arm is confirmed settled there), waits for a fresh
/// marker_pose message published after that point, and takes exactly one
/// sample from it. This replaces an earlier passive-timer design (accept
/// whatever arrived every min_sample_interval_sec, regardless of whether
/// the arm was mid-motion) that produced motion-blur-corrupted samples —
/// see error-mitigation.md #19 and progress.md's Feature Additions entry
/// on signal-based sync.
///
/// trajectory_planner is never told calibration exists — it only ever
/// sees ordinary ~/trace_path/~/get_polygon_waypoints calls, so it stays a
/// dumb mover with no calibration awareness. All orchestration logic
/// (waypoint iteration, sample timing, averaging, broadcast) lives here.
///
/// Runs the whole per-goal sequence on a dedicated thread (spawned from
/// handleAccepted), not inline in an action-server callback or the
/// marker_pose subscription callback — both would block the executor that
/// also needs to process the ~/trace_path service-client response and
/// incoming marker_pose messages this loop depends on.
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

  /// Caches the latest message (with its receipt time) and notifies
  /// sample_cv_ — see requestSampleAfterSettling.
  void markerPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr & msg);

  /// Accepts a new goal unless calibration is already in progress.
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Calibrate::Goal> goal);

  /// Always accepts cancellation requests — executeCalibration polls
  /// goal_handle->is_canceling() between waypoints.
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleCalibrate> goal_handle);

  /// Spawns a detached thread running executeCalibration(goal_handle) —
  /// rclcpp_action requires handleAccepted to return quickly, not block.
  void handleAccepted(const std::shared_ptr<GoalHandleCalibrate> goal_handle);

  /// The actual orchestration sequence, run on its own thread:
  /// 1. Call ~/get_polygon_waypoints once.
  /// 2. For each of config_.num_samples waypoints needed (cycling through
  ///    the returned list if num_samples exceeds its length): call
  ///    ~/trace_path with that single waypoint, wait for a fresh
  ///    marker_pose (see requestSampleAfterSettling), record one sample,
  ///    publish feedback. Aborts (goal_handle->abort) on any failure
  ///    (waypoint fetch, trace_path, or sample-wait timeout) or on
  ///    cancellation.
  /// 3. On success, calls finishCalibration() to average + broadcast +
  ///    complete the goal.
  void executeCalibration(const std::shared_ptr<GoalHandleCalibrate> goal_handle);

  /// Blocks (up to config_.sample_wait_timeout_sec) until a marker_pose
  /// message is received whose receipt time is after `after`, then
  /// returns it. Returns std::nullopt on timeout. This wait is what
  /// guarantees a sample reflects the arm's settled pose — the previous
  /// design sampled whatever the most recently cached message was,
  /// regardless of whether it predated the settle.
  std::optional<geometry_msgs::msg::PoseStamped> waitForFreshMarkerPose(
    const rclcpp::Time & after);

  /// Chains one fresh marker_pose (camera_frame -> marker, from the
  /// detector) with the live known_chain_frame -> marker_frame TF into
  /// one sample of known_chain_frame -> camera, and appends it to
  /// collected_positions_/collected_orientations_. Returns false (logs
  /// the error) if the TF lookup fails.
  bool recordSample(const geometry_msgs::msg::PoseStamped & marker_pose);

  /// Averages collected_positions_ (arithmetic mean) and
  /// collected_orientations_ (via averaging_method_), broadcasts
  /// known_chain_frame -> the camera frame (from the most recent sample's
  /// header.frame_id) as a static TF, and completes goal_handle with the
  /// result (see Calibrate.action). Logs the orientation spread metrics.
  /// Clears both collected_ vectors.
  void finishCalibration(const std::shared_ptr<GoalHandleCalibrate> & goal_handle);

  CalibrationBroadcasterConfig config_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;
  /// Selected once at construction from config_'s priorities — see
  /// selectAveragingMethod.
  OrientationAveragingMethod averaging_method_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr marker_pose_sub_;
  rclcpp_action::Server<Calibrate>::SharedPtr calibrate_action_server_;
  rclcpp::Client<visual_calibration_msgs::srv::GetPolygonWaypoints>::SharedPtr
    get_polygon_waypoints_client_;
  rclcpp::Client<visual_calibration_msgs::srv::TracePath>::SharedPtr trace_path_client_;

  /// Guards latest_marker_pose_/latest_marker_pose_stamp_, notified by
  /// markerPoseCallback and waited on by waitForFreshMarkerPose.
  std::mutex sample_mutex_;
  std::condition_variable sample_cv_;
  geometry_msgs::msg::PoseStamped latest_marker_pose_;
  rclcpp::Time latest_marker_pose_stamp_;

  std::vector<geometry_msgs::msg::Vector3> collected_positions_;
  std::vector<tf2::Quaternion> collected_orientations_;
  /// The most recent sample's camera frame_id — carried through to the
  /// final broadcast's child_frame_id.
  geometry_msgs::msg::PoseStamped last_sample_;
};

}  // namespace aruco_perception

#endif  // ARUCO_PERCEPTION__CALIBRATION_BROADCASTER_NODE_HPP_
