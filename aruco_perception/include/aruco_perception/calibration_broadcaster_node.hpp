#ifndef ARUCO_PERCEPTION__CALIBRATION_BROADCASTER_NODE_HPP_
#define ARUCO_PERCEPTION__CALIBRATION_BROADCASTER_NODE_HPP_

#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose.hpp>
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
  /// Number of samples taken during the polygon phase — one sample per
  /// waypoint visited, cycling through the returned polygon waypoints if
  /// this exceeds their count. Named distinctly from the random phase's
  /// own count (random_phase_samples) since the two phases now run
  /// sequentially, not as one undifferentiated loop — see
  /// CalibrationBroadcasterNode's class doc comment for the two-phase
  /// design (2026-07-22 redesign).
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

  // --- Random phase (2026-07-22 redesign) ---
  /// Number of samples to collect during the random phase, after the
  /// polygon phase completes — see runRandomPhase.
  int random_phase_samples = 8;
  /// Maximum straight-line distance (meters) a random candidate pose may
  /// be from the center pose (the same center the polygon phase used —
  /// see GetPolygonWaypoints.srv's center_pose field), checked BEFORE
  /// moving there. A simple stateless per-candidate check, not a
  /// cumulative/path-history one.
  double random_phase_max_offset_m = 0.10;
  /// If a random candidate's move succeeds but the marker isn't visible
  /// there, the attempt is discarded (not counted) and a new candidate is
  /// generated from the center pose — this caps how many consecutive
  /// discards are allowed before runRandomPhase gives up and aborts the
  /// whole calibration run (a safety bound against an unlucky/impossible
  /// random-offset run, not expected to be hit in normal operation).
  int random_phase_max_consecutive_failures = 20;

  // --- Early-stop (2026-07-22 redesign) ---
  /// Position-spread threshold (cm): a sample's position is considered
  /// "in agreement" with the running average if it's within this distance
  /// of the mean of all samples collected so far. Both this AND
  /// orientation_spread_tolerance_deg must hold for a sample to count
  /// toward stable_agreement_count.
  double position_spread_tolerance_cm = 2.0;
  /// Orientation-spread threshold (degrees) — see
  /// position_spread_tolerance_cm; this is the angular equivalent,
  /// checked against the running orientation average (via
  /// averageQuaternions, not the final one-shot call in finishCalibration).
  double orientation_spread_tolerance_deg = 5.0;
  /// Number of samples (not necessarily consecutive) that must fall
  /// within both spread tolerances of the running average, counted from
  /// the moment the polygon phase completes onward, before calibration
  /// stops collecting early and proceeds straight to finishCalibration().
  /// Tunable up if real-world noise causes false-early stops.
  int stable_agreement_count = 2;
};

/// Orchestrates calibration: fetches waypoints AND their center pose from
/// trajectory_planner (~/get_polygon_waypoints, read-only — see
/// GetPolygonWaypoints.srv's center_pose field), then runs TWO sequential
/// sample-collection phases (2026-07-22 redesign):
///   1. Polygon phase (runPolygonPhase) — visits the polygon corners
///      (2 full passes, config_.num_samples total).
///   2. Random phase (runRandomPhase) — config_.random_phase_samples more
///      samples at randomized X/Y/Z offsets from the SAME center pose
///      (randomPoseNear), each capped at random_phase_max_offset_m and
///      visibility-checked before counting.
/// Both phases share the same per-sample sequence the original single-
/// phase design used: calls trajectory_planner's ~/trace_path with a
/// single waypoint (blocking until the arm is confirmed settled there),
/// waits for a fresh marker_pose message published after that point, and
/// takes exactly one sample from it. This settle-then-sample sync
/// replaces an earlier passive-timer design (accept whatever arrived
/// every min_sample_interval_sec, regardless of whether the arm was
/// mid-motion) that produced motion-blur-corrupted samples — see
/// error-mitigation.md #19 and progress.md's Feature Additions entry on
/// signal-based sync.
///
/// After every recorded sample (either phase), checks
/// stableAgreementReached() — if the running position/orientation spread
/// has stayed within tolerance for enough samples, collection stops
/// immediately (early-stop) rather than always running the full
/// polygon+random count.
///
/// trajectory_planner is never told calibration exists — it only ever
/// sees ordinary ~/trace_path/~/get_polygon_waypoints calls, so it stays a
/// dumb mover with no calibration awareness. All orchestration logic
/// (phase sequencing, waypoint/random-pose generation, sample timing,
/// early-stop, averaging, broadcast) lives here.
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

  /// The actual orchestration sequence, run on its own thread (2026-07-22
  /// redesign — two phases, not one undifferentiated loop):
  /// 1. Call ~/get_polygon_waypoints once — gets both the polygon corner
  ///    waypoints AND the center pose they were generated around (see
  ///    GetPolygonWaypoints.srv's center_pose field).
  /// 2. Polygon phase (runPolygonPhase): visits the polygon corners for 2
  ///    full passes (config_.num_samples total, cycling through the
  ///    corner list same as before), one sample per waypoint.
  /// 3. Random phase (runRandomPhase): config_.random_phase_samples
  ///    additional samples at randomized offsets from the SAME center
  ///    pose, varying X/Y/Z (see randomPoseNear), each visibility-checked
  ///    before counting.
  /// Both phases check the early-stop condition (see
  /// stableAgreementReached) after every recorded sample and stop
  /// collecting immediately if it's reached, regardless of which phase is
  /// active. Aborts (goal_handle->abort) on any failure (waypoint fetch,
  /// trace_path, or sample-wait timeout) or on cancellation. On success
  /// (either the full sample count was collected, or early-stop
  /// triggered), calls finishCalibration() to average + broadcast +
  /// complete the goal.
  void executeCalibration(const std::shared_ptr<GoalHandleCalibrate> goal_handle);

  /// Polygon phase: visits `waypoints` (the polygon corners from
  /// ~/get_polygon_waypoints) for 2 full passes, cycling via modulo same
  /// as the original single-phase design, up to config_.num_samples
  /// samples — fewer if the early-stop condition triggers first (see
  /// stableAgreementReached, checked after every recorded sample). Shares
  /// the same trace_path + waitForFreshMarkerPose + recordSample sequence
  /// the original design used per-waypoint. Returns false (and sets
  /// *out_result with a failure Calibrate::Result, goal_handle NOT yet
  /// aborted — the caller does that) on the first hard failure
  /// (trace_path, sample-wait timeout, TF lookup) or cancellation; true
  /// otherwise (including "stopped early via early-stop" — check
  /// stopped_early to distinguish from "collected the full count").
  bool runPolygonPhase(
    const std::shared_ptr<GoalHandleCalibrate> & goal_handle,
    const std::vector<geometry_msgs::msg::Pose> & waypoints,
    std::shared_ptr<Calibrate::Result> & out_result,
    bool & stopped_early);

  /// Random phase: generates config_.random_phase_samples valid samples
  /// (fewer if early-stop triggers first) at randomized offsets from
  /// center_pose (see randomPoseNear), each capped at
  /// config_.random_phase_max_offset_m straight-line distance from
  /// center_pose. For each candidate: moves there via trace_path, checks
  /// marker visibility (isMarkerVisibleNow) — if visible, records the
  /// sample and continues; if the move itself fails, that's a hard
  /// failure (same as the polygon phase); if the move succeeds but the
  /// marker isn't visible, the attempt is discarded (not counted), the
  /// arm moves back to center_pose immediately (no point probing further
  /// out when not visible at all), and a new candidate is generated —
  /// bounded by config_.random_phase_max_consecutive_failures consecutive
  /// discards before giving up as a hard failure. Same out_result/
  /// stopped_early/return-value contract as runPolygonPhase.
  bool runRandomPhase(
    const std::shared_ptr<GoalHandleCalibrate> & goal_handle,
    const geometry_msgs::msg::Pose & center_pose,
    int samples_already_collected,
    std::shared_ptr<Calibrate::Result> & out_result,
    bool & stopped_early);

  /// Generates a uniformly-random offset pose from center_pose, varying
  /// X/Y/Z independently within +-config_.random_phase_max_offset_m
  /// (checked as a straight-line distance cap from center_pose before
  /// returning — a candidate exceeding the cap is rejected and re-rolled
  /// internally, not returned for the caller to check), keeping
  /// center_pose's orientation unchanged. Same tf2::Transform
  /// center * offset pattern already used by
  /// TrajectoryPlanner::polygonWaypointsAroundStandoff and
  /// CalibrationOrchestratorNode::probeDirectionVisible.
  geometry_msgs::msg::Pose randomPoseNear(
    const geometry_msgs::msg::Pose & center_pose, double max_offset_m) const;

  /// Sends a single-waypoint ~/trace_path request (config_.planning_mode)
  /// and blocks for the response. Shared by both phases (the original
  /// design inlined this in executeCalibration's loop; split out here so
  /// runPolygonPhase/runRandomPhase/runRandomPhase's return-to-center step
  /// don't duplicate it). Returns false if the service isn't available or
  /// the call fails.
  bool tracePathBlocking(const geometry_msgs::msg::Pose & target);

  /// Blocks (up to config_.sample_wait_timeout_sec) until a marker_pose
  /// message is received whose receipt time is after `after`, then
  /// returns it. Returns std::nullopt on timeout. This wait is what
  /// guarantees a sample reflects the arm's settled pose — the previous
  /// design sampled whatever the most recently cached message was,
  /// regardless of whether it predated the settle.
  std::optional<geometry_msgs::msg::PoseStamped> waitForFreshMarkerPose(
    const rclcpp::Time & after);

  /// Like waitForFreshMarkerPose, but only checks for visibility (doesn't
  /// need/return the pose itself) — used by the random phase's
  /// per-candidate visibility check, mirroring
  /// CalibrationOrchestratorNode::isMarkerVisibleAfter's polling pattern
  /// (a probe move to a genuinely-invisible position must time out
  /// gracefully, not hang, so this polls rather than blocking on the
  /// condition variable the way waitForFreshMarkerPose does).
  bool isMarkerVisibleNow(const rclcpp::Time & after);

  /// Chains one fresh marker_pose (camera_frame -> marker, from the
  /// detector) with the live known_chain_frame -> marker_frame TF into
  /// one sample of known_chain_frame -> camera, and appends it to
  /// collected_positions_/collected_orientations_. Returns false (logs
  /// the error) if the TF lookup fails.
  bool recordSample(const geometry_msgs::msg::PoseStamped & marker_pose);

  /// Early-stop check (2026-07-22 redesign): called after every
  /// recordSample() success, in both phases. Computes the running
  /// position spread (max distance, in cm, of any collected sample's
  /// position from the arithmetic mean of all collected positions so
  /// far) and running orientation spread (max_spread_deg from
  /// averageQuaternions(collected_orientations_, averaging_method_) —
  /// safe to call mid-run, it's a pure function over whatever's collected
  /// so far, not just at finishCalibration() time). If BOTH are within
  /// their respective tolerances (config_.position_spread_tolerance_cm/
  /// orientation_spread_tolerance_deg), increments
  /// stable_agreement_count_ (a running, non-consecutive count — NOT
  /// reset when a sample falls outside tolerance) and returns true once
  /// that counter reaches config_.stable_agreement_count. Does nothing
  /// (returns false) if fewer than 2 samples are collected yet (spread is
  /// meaningless with only 1 sample).
  bool stableAgreementReached();

  /// Averages collected_positions_ (arithmetic mean) and
  /// collected_orientations_ (via averaging_method_), broadcasts
  /// known_chain_frame -> the camera frame (from the most recent sample's
  /// header.frame_id) as a static TF, and completes goal_handle with the
  /// result (see Calibrate.action). Logs the orientation spread metrics.
  /// Clears both collected_ vectors AND resets stable_agreement_count_ for
  /// the next run.
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
  /// Running, non-consecutive count of samples found "in agreement" with
  /// the running average — see stableAgreementReached. Reset to 0 only in
  /// finishCalibration() (i.e. once per calibration run), NOT when a
  /// sample falls outside tolerance.
  int stable_agreement_count_ = 0;
  /// Random-offset generation (randomPoseNear) needs a seeded engine —
  /// member rather than a function-local static so it's not shared/reset
  /// oddly across concurrent goals (executeCalibration runs one goal at a
  /// time per handleGoal's doc comment, but keeping this as ordinary
  /// instance state is simpler than reasoning about static init order).
  mutable std::mt19937 random_engine_{std::random_device{}()};
};

}  // namespace aruco_perception

#endif  // ARUCO_PERCEPTION__CALIBRATION_BROADCASTER_NODE_HPP_
