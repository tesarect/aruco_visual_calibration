#ifndef ARUCO_PERCEPTION__CALIBRATION_BROADCASTER_NODE_HPP_
#define ARUCO_PERCEPTION__CALIBRATION_BROADCASTER_NODE_HPP_

#include <array>
#include <condition_variable>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visual_calibration_msgs/action/calibrate.hpp>
#include <visual_calibration_msgs/srv/detect_marker_once.hpp>
#include <visual_calibration_msgs/srv/get_polygon_waypoints.hpp>
#include <visual_calibration_msgs/srv/signal_inference_server.hpp>
#include <visual_calibration_msgs/srv/trace_path.hpp>

#include "aruco_perception/orientation_averaging.hpp"

namespace aruco_perception
{

/// Tuning for CalibrationBroadcasterNode, loaded from a parameter file.
///
/// known_chain_frame/marker_frame name the TF chain already known from the
/// robot's own kinematics (joint states). Which frame is "known" and which
/// is the fixed unknown being solved for (the camera) depends on the
/// physical mounting: with a wrist-mounted camera, marker and camera both
/// ride the arm and base_link->marker is known; with a wall/ceiling-mounted
/// camera, base_link->camera is the fixed unknown and the arm carries the
/// marker instead. The node's logic is identical either way — only these
/// two param values change per environment.
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

  /// Compensating rotation-only offset applied to marker_frame's own
  /// orientation in recordSample(), for deployments where marker_frame is
  /// not a purpose-built child frame with a measured mount orientation
  /// (unlike, e.g., a gripper's dedicated aruco joint with a measured rpy).
  /// [roll, pitch, yaw] degrees, applied to known_to_marker's rotation only
  /// (translation untouched) before combining with marker_to_camera in
  /// recordSample(). Default [0,0,0] is a no-op. This is a manual
  /// best-estimate correction, not a physical re-measurement — see
  /// recordSample() for the exact composition order.
  std::array<double, 3> marker_frame_orientation_offset_rpy_deg{0.0, 0.0, 0.0};
  /// Appended to the detector's camera frame_id to form the broadcast TF's
  /// child_frame_id (e.g. "wrist_rgbd_camera_depth_optical_frame" becomes
  /// "..._calibrated"). Broadcasting under the exact same name as an
  /// existing URDF-declared frame would conflict with it in the TF tree
  /// (two disagreeing publishers for one frame); this suffix keeps the
  /// computed result distinct from any physically-declared camera frame.
  std::string broadcast_frame_suffix = "_calibrated";
  /// Number of samples taken during the polygon phase, one sample per
  /// waypoint visited, cycling through the returned polygon waypoints if
  /// this exceeds their count. Distinct from the random phase's own count
  /// (random_phase_samples) since the two phases run sequentially — see
  /// CalibrationBroadcasterNode's class doc comment.
  int num_samples = 10;
  /// How long to wait for a fresh marker_pose message (published after the
  /// arm is confirmed settled at a waypoint — see
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
  /// Implemented (see orientation_averaging.cpp's markleyAverage()), but
  /// left at 0 in both sim and real configs by default — an opt-in
  /// alternative to kSumNormalize, not yet a default-behavior change.
  int orientation_markley_priority = 0;

  // --- Random phase ---
  /// Number of samples to collect during the random phase, after the
  /// polygon phase completes — see runRandomPhase.
  int random_phase_samples = 8;
  /// Maximum straight-line distance (meters) a random candidate pose may
  /// be from the center pose (the same center the polygon phase used —
  /// see GetPolygonWaypoints.srv's center_pose field), checked before
  /// moving there. A stateless per-candidate check, not a cumulative one.
  double random_phase_max_offset_m = 0.10;
  /// If a random candidate's move succeeds but the marker isn't visible
  /// there, the attempt is discarded and a new candidate is generated from
  /// the center pose. Caps how many consecutive discards are allowed
  /// before runRandomPhase gives up and aborts the run — a safety bound
  /// against an unlucky/impossible random-offset run.
  int random_phase_max_consecutive_failures = 20;

  // --- Early stop ---
  /// Position-spread threshold (cm): a sample's position is "in agreement"
  /// with the running average if it's within this distance of the mean of
  /// all samples collected so far. Both this and
  /// orientation_spread_tolerance_deg must hold for a sample to count
  /// toward stable_agreement_count.
  double position_spread_tolerance_cm = 2.0;
  /// Orientation-spread threshold (degrees), checked against the running
  /// orientation average (via averageQuaternions, not the one-shot call in
  /// finishCalibration).
  double orientation_spread_tolerance_deg = 5.0;
  /// Number of samples (not necessarily consecutive) that must fall within
  /// both spread tolerances of the running average, counted from the
  /// moment the polygon phase completes, before calibration stops
  /// collecting early and proceeds to finishCalibration().
  int stable_agreement_count = 2;

  // --- Orientation sweep phase ---
  /// When true, runs runOrientationSweepPhase() once after polygon/random
  /// sampling completes (whether by early stop or full count) and before
  /// finishCalibration(). Adds orientation diversity beyond the polygon/
  /// random phases' position-only offsets (see randomPoseNear, which never
  /// varies orientation).
  bool orientation_sweep_enabled = false;
  /// Pitch/roll offset magnitude (degrees) used for all 4 sweep probes
  /// (pitch down, pitch up, roll left, roll right) — see
  /// runOrientationSweepPhase.
  double orientation_sweep_angle_deg = 5.0;

  // --- Outlier rejection ---
  /// When true, finishCalibration() discards any collected sample whose
  /// position or orientation deviation from the (unfiltered) mean exceeds
  /// outlier_position_threshold_cm / outlier_orientation_threshold_deg,
  /// before computing the final average. A sample is discarded if either
  /// threshold is exceeded. Threshold-based, not fixed-worst-N: a clean
  /// run where every sample is already within both thresholds discards
  /// nothing.
  bool outlier_rejection_enabled = true;
  double outlier_position_threshold_cm = 2.0;
  double outlier_orientation_threshold_deg = 5.0;

  // --- Dual sampling per waypoint ---
  /// Number of samples taken at each polygon/random-phase waypoint before
  /// moving to the next one (no additional move between them — same
  /// settled pose). Default 2 mitigates a single bad/missed detection
  /// being that waypoint's only data point. Samples from the same waypoint
  /// are pooled with every other sample; outlier_rejection is what sorts
  /// out any disagreement between them.
  int samples_per_waypoint = 2;

  // --- Per-waypoint on-demand hybrid detection ---
  /// When true, each polygon/random-phase waypoint's sample(s) come from
  /// exactly one ~/detect_marker_once call to yolo_marker_bridge_node
  /// (DetectMarkerOnce.srv: YOLO crop + image-enhancement cascade +
  /// classical ArUco + solvePnP, run once per call) instead of the
  /// continuous marker_pose topic (waitForFreshMarkerPose) — see
  /// sampleOnceAtCurrentWaypoint for the full mechanism. This lets YOLO's
  /// crop + enhancement cascade run ahead of classical corner-finding
  /// while only paying its cost once per waypoint rather than
  /// continuously. Each waypoint's single detection call is bracketed with
  /// SIGCONT/SIGSTOP (via ~/signal_inference_server on
  /// calibration_orchestrator_node) so the model process is live only for
  /// the duration of that call.
  ///
  /// This flag is not stored in this struct — it is read live via
  /// get_parameter_or() at the point of use (sampleOnceAtCurrentWaypoint),
  /// the same live-read convention use_clustering_average uses, so it can
  /// be flipped mid-session via set_parameters (e.g. from the web app's
  /// "Hybrid ArUco Detection" switch) with no node restart.

  /// Bounded wait (seconds) on the ~/detect_marker_once future in
  /// sampleOnceAtCurrentWaypoint, so a hung request cannot block forever.
  /// A timeout here is treated as a failed sample for that waypoint only
  /// (soft-fail, same as "no marker found"), not a hard abort by itself.
  double detect_call_timeout_sec = 30.0;

  // --- Discard-and-continue on a failed waypoint sample ---
  /// When min_samples_to_finish > 0, a failed first-attempt sample at a
  /// waypoint is soft-failed (logged, skipped, move to the next waypoint)
  /// instead of hard-aborting the whole run. The run only hard-fails if,
  /// once all polygon+random waypoints have been attempted, fewer than
  /// this many total samples were collected. Default 0 preserves strict
  /// behavior (any first-attempt failure hard-aborts immediately) — this
  /// is an opt-in relaxation intended mainly for real-world detection
  /// misses.
  int min_samples_to_finish = 0;

  /// Number of attempts per sample — see sampleWithRetry, which every
  /// sampling call site (center-pose sample, and every polygon/random
  /// waypoint) goes through. Each retry takes a genuinely fresh frame, not
  /// a re-check of a stale result. 1 = no retry; a value above 1 gives a
  /// transient one-bad-frame miss a chance to recover before falling
  /// through to min_samples_to_finish's discard-and-continue handling.
  int cal_ready_hybrid_marker_detection_retry = 3;

  // --- Clustering-based position+orientation averaging ---
  /// clustering_bucket_size_cm/clustering_bucket_angle_deg are cached in
  /// config_ like every other tuning constant (restart-only to change).
  /// use_clustering_average is deliberately not a field here — see
  /// finishCalibration() for why it must be read live via get_parameter()
  /// at the point of use rather than cached, so it can be toggled from the
  /// web UI without a node restart, unlike the other fields in this
  /// struct.
  ///
  /// Bucket/offset tolerance (cm) for treating two samples' positions as
  /// "the same cluster" when use_clustering_average is true — see
  /// computeClusteredPose(). Same default as
  /// position_spread_tolerance_cm/outlier_position_threshold_cm (2.0cm),
  /// but independently tunable.
  double clustering_bucket_size_cm = 2.0;
  /// Angular tolerance (degrees) for treating two samples' orientations as
  /// "the same cluster" — see computeClusteredPose(). Both this and
  /// clustering_bucket_size_cm must hold for two samples to be grouped
  /// together. Same default as orientation_spread_tolerance_deg/
  /// outlier_orientation_threshold_deg (5.0deg), but independently
  /// tunable.
  double clustering_bucket_angle_deg = 5.0;

  // --- Per-run yaw/roll clamp ---
  /// For a rigidly wall/ceiling-mounted camera, yaw and roll relative to
  /// base_link are physically fixed by the mount and only pitch (tilt
  /// up/down) is expected to vary. aruco_detector_node does no corner/pose
  /// smoothing, so every frame's raw ArUco-derived orientation flows
  /// straight into collected_orientations_ — any yaw/roll variation
  /// observed across a run's samples is therefore corner-detection noise,
  /// not real orientation change, when the mount is in fact rigid.
  ///
  /// When true, finishCalibration() computes a circular mean of yaw and of
  /// roll across all samples collected this run (self-averaged rather
  /// than a hardcoded constant, since the true mount yaw/roll may be
  /// unmeasured), then re-encodes every sample's orientation as
  /// (mean_roll, that sample's own original pitch, mean_yaw) before
  /// rejectOutliers()/averaging runs — see clampYawRoll() for the full
  /// algorithm. Pitch is left untouched per-sample since it is the one
  /// axis that can carry real signal.
  ///
  /// Default false: this is an opt-in correction that must not silently
  /// change existing behavior. Restart-only, unlike use_clustering_average
  /// above — only the on/off switch is restart-only; the clamp's own
  /// per-run mean computation still runs fresh every finishCalibration()
  /// call.
  bool yaw_roll_clamp_enabled = false;

  // --- Forced yaw/roll override ---
  /// Optional test hook: when the mount's true yaw/roll are known and
  /// physically measured, this lets clampYawRoll() use those known
  /// constants directly instead of re-deriving them from this run's own
  /// samples via circular mean. NaN (default, unset) on either field means
  /// "use the circular-mean behavior for that axis"; set both in degrees
  /// to bypass the mean computation entirely.
  double yaw_roll_clamp_forced_yaw_deg = std::numeric_limits<double>::quiet_NaN();
  double yaw_roll_clamp_forced_roll_deg = std::numeric_limits<double>::quiet_NaN();
};

/**
 * Orchestrates the calibration action: fetches waypoints and their center
 * pose from trajectory_planner (~/get_polygon_waypoints, read-only — see
 * GetPolygonWaypoints.srv's center_pose field), then runs two sequential
 * sample-collection phases:
 *   1. Polygon phase (runPolygonPhase) — visits the polygon corners
 *      (2 full passes, config_.num_samples total).
 *   2. Random phase (runRandomPhase) — config_.random_phase_samples more
 *      samples at randomized X/Y/Z offsets from the same center pose
 *      (randomPoseNear), each capped at random_phase_max_offset_m and
 *      visibility-checked before counting.
 *
 * Both phases share the same per-sample sequence: call trajectory_planner's
 * ~/trace_path with a single waypoint (blocking until the arm is confirmed
 * settled there), wait for a fresh marker_pose message published after
 * that point, and take exactly one sample from it. This settle-then-sample
 * synchronization avoids motion-blur-corrupted samples that a passive
 * fixed-interval sampler would be vulnerable to.
 *
 * After every recorded sample (either phase), checks
 * stableAgreementReached() — if the running position/orientation spread
 * has stayed within tolerance for enough samples, collection stops
 * immediately (early stop) rather than always running the full
 * polygon+random count.
 *
 * trajectory_planner is never told calibration exists — it only ever sees
 * ordinary ~/trace_path/~/get_polygon_waypoints calls, so it stays a dumb
 * mover with no calibration awareness. All orchestration logic (phase
 * sequencing, waypoint/random-pose generation, sample timing, early stop,
 * averaging, broadcast) lives here.
 *
 * Runs the whole per-goal sequence on a dedicated thread (spawned from
 * handleAccepted), not inline in an action-server callback or the
 * marker_pose subscription callback — both would block the executor that
 * also needs to process the ~/trace_path service-client response and
 * incoming marker_pose messages this loop depends on.
 *
 * Position: arithmetic mean of all samples. Orientation: averaged via
 * whichever OrientationAveragingMethod selectAveragingMethod picks from
 * config_'s priorities (kSumNormalize by default; kMarkley reserved for a
 * more robust average later — see orientation_averaging.hpp). Both
 * resulting spread metrics are included in the action result and logged
 * as a signal for whether the average is trustworthy.
 */

/// Result of computeClusteredPose(): both position and orientation of the
/// winning cluster's members, since clustering groups on both — position-
/// only clustering can still let an outlier orientation drag the
/// quaternion average even when position converges well.
struct ClusteredPose
{
  geometry_msgs::msg::Vector3 position;
  tf2::Quaternion orientation;
  /// Local indices (into the `indices` vector passed to
  /// computeClusteredPosition(), not collected_positions_ directly) of the
  /// winning cluster's members, so the caller can compute a
  /// post-clustering spread/is_high_confidence check against exactly the
  /// samples that contributed to this result.
  std::vector<size_t> member_indices;
};

class CalibrationBroadcasterNode : public rclcpp::Node
{
public:
  using Calibrate = visual_calibration_msgs::action::Calibrate;
  using GoalHandleCalibrate = rclcpp_action::ServerGoalHandle<Calibrate>;

  CalibrationBroadcasterNode();

private:
  /// Grants the small RAII guard in calibration_broadcaster_node.cpp
  /// (which guarantees saveDebugImageGrid() and, if the run was in hybrid
  /// mode, a resuming SIGCONT both run on every executeCalibration exit
  /// path) access to the otherwise-private
  /// saveDebugImageGrid()/signalInferenceServerViaOrchestrator() below.
  friend struct EndOfRunCleanupGuard;

  CalibrationBroadcasterConfig loadConfigFromParams() const;

  /// Total sample count a full (non-early-stopped) run will collect, used
  /// for feedback's samples_total field: 1 (center) +
  /// config_.num_samples * config_.samples_per_waypoint (polygon) +
  /// config_.random_phase_samples * config_.samples_per_waypoint (random)
  /// + (4 if config_.orientation_sweep_enabled, else 0). Centralized here
  /// so the formula isn't duplicated at every call site.
  int totalSamplesTarget() const;

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

  /// The full orchestration sequence, run on its own thread:
  /// 1. Call ~/get_polygon_waypoints once — gets both the polygon corner
  ///    waypoints and the center pose they were generated around (see
  ///    GetPolygonWaypoints.srv's center_pose field).
  /// 2. Polygon phase (runPolygonPhase): visits the polygon corners for 2
  ///    full passes (config_.num_samples total, cycling through the
  ///    corner list), one sample per waypoint.
  /// 3. Random phase (runRandomPhase): config_.random_phase_samples
  ///    additional samples at randomized offsets from the same center
  ///    pose, varying X/Y/Z (see randomPoseNear), each visibility-checked
  ///    before counting.
  /// Both phases check the early-stop condition (see
  /// stableAgreementReached) after every recorded sample and stop
  /// collecting immediately if it's reached, regardless of which phase is
  /// active. Aborts (goal_handle->abort) on any failure (waypoint fetch,
  /// trace_path, or sample-wait timeout) or on cancellation. On success
  /// (either the full sample count was collected, or early stop
  /// triggered), calls finishCalibration() to average + broadcast +
  /// complete the goal.
  void executeCalibration(const std::shared_ptr<GoalHandleCalibrate> goal_handle);

  /// Polygon phase: visits `waypoints` (the polygon corners from
  /// ~/get_polygon_waypoints) for 2 full passes, cycling via modulo, up to
  /// config_.num_samples samples — fewer if the early-stop condition
  /// triggers first (see stableAgreementReached, checked after every
  /// recorded sample). Shares the trace_path + waitForFreshMarkerPose +
  /// recordSample sequence with the random phase, taking
  /// config_.samples_per_waypoint samples per settled pose (no additional
  /// move between them) before advancing to the next waypoint; early stop
  /// is checked after every individual sample, so it can trigger
  /// mid-waypoint. Returns false (and sets *out_result with a failure
  /// Calibrate::Result — the caller aborts goal_handle) on the first hard
  /// failure (trace_path, sample-wait timeout, TF lookup) or cancellation;
  /// true otherwise (including "stopped early" — check stopped_early to
  /// distinguish from "collected the full count").
  bool runPolygonPhase(
    const std::shared_ptr<GoalHandleCalibrate> & goal_handle,
    const std::vector<geometry_msgs::msg::Pose> & waypoints,
    std::shared_ptr<Calibrate::Result> & out_result,
    bool & stopped_early);

  /// Random phase: generates config_.random_phase_samples valid samples
  /// (fewer if early stop triggers first) at randomized offsets from
  /// center_pose (see randomPoseNear), each capped at
  /// config_.random_phase_max_offset_m straight-line distance from
  /// center_pose. For each candidate: moves there via trace_path, checks
  /// marker visibility (isMarkerVisibleNow) — if visible, records the
  /// sample and continues; if the move itself fails, that's a hard
  /// failure (same as the polygon phase); if the move succeeds but the
  /// marker isn't visible, the attempt is discarded, the arm moves back
  /// to center_pose immediately, and a new candidate is generated —
  /// bounded by config_.random_phase_max_consecutive_failures consecutive
  /// discards before giving up as a hard failure. Like runPolygonPhase,
  /// takes config_.samples_per_waypoint samples per successfully-visible
  /// candidate before moving on. Same out_result/stopped_early/
  /// return-value contract as runPolygonPhase.
  bool runRandomPhase(
    const std::shared_ptr<GoalHandleCalibrate> & goal_handle,
    const geometry_msgs::msg::Pose & center_pose,
    int samples_already_collected,
    std::shared_ptr<Calibrate::Result> & out_result,
    bool & stopped_early);

  /// Orientation sweep phase: runs once, after polygon/random sampling is
  /// otherwise done (early stop or full count), only when
  /// config_.orientation_sweep_enabled is true. Returns to cal_ready_pose,
  /// then probes 4 rotational offsets from its orientation — pitch down,
  /// pitch up, roll left, roll right, each config_.orientation_sweep_angle_deg
  /// degrees, each independently offset from cal_ready_pose's own
  /// orientation (not cumulative) — taking one sample per probe that
  /// successfully lands with the marker visible. A probe whose move fails
  /// or whose marker isn't visible is skipped (logged, not counted, not a
  /// hard failure), the same treatment runRandomPhase gives an invisible
  /// marker at a candidate pose. Returns to cal_ready_pose again at the
  /// end, leaving the arm in a known pose before finishCalibration().
  /// Samples are appended to the same collected_positions_/
  /// collected_orientations_ pool as every other sample.
  void runOrientationSweepPhase(
    const std::shared_ptr<GoalHandleCalibrate> & goal_handle,
    const geometry_msgs::msg::Pose & cal_ready_pose,
    int samples_already_collected);

  /// Builds a geometry_msgs::msg::Pose from collected_positions_.back()/
  /// collected_orientations_.back() (the just-recorded sample), for
  /// populating Calibrate::Feedback::latest_sample_pose. Also broadcasts
  /// the same pose as a transient (non-static) TF frame, known_chain_frame
  /// -> "camera_calibration_sample" (a single frame replaced every call,
  /// not accumulated — see sample_tf_broadcaster_'s doc comment for why a
  /// plain TransformBroadcaster, not StaticTransformBroadcaster, is used),
  /// so RViz shows the live in-progress candidate pose updating sample by
  /// sample. This is the primary, always-on visualization; the web app's
  /// latest_sample_pose feedback field is an additional path for
  /// browser-side visualization. Must be called immediately after every
  /// recordSample() success, before building/publishing that sample's
  /// feedback message.
  geometry_msgs::msg::Pose broadcastLatestSamplePose();

  /// Builds a pose offset from `base_pose` by a pure rotation (pitch or
  /// roll) around base_pose's own local axis, position unchanged — same
  /// tf2::Transform (base * offset) composition pattern as randomPoseNear,
  /// but rotation-only (translation zero) instead of translation-only.
  /// is_pitch selects which local axis the angle is applied around — see
  /// runOrientationSweepPhase's call sites and this method's .cpp doc
  /// comment for the axis mapping, confirmed against trajectory_planner's
  /// end_effector_frame convention.
  geometry_msgs::msg::Pose rotatedPoseNear(
    const geometry_msgs::msg::Pose & base_pose, double angle_deg, bool is_pitch) const;

  /// Generates a uniformly-random offset pose from center_pose, varying
  /// X/Y/Z independently within +-config_.random_phase_max_offset_m
  /// (checked as a straight-line distance cap from center_pose before
  /// returning — a candidate exceeding the cap is rejected and re-rolled
  /// internally), keeping center_pose's orientation unchanged. Same
  /// tf2::Transform center * offset pattern already used by
  /// TrajectoryPlanner::polygonWaypointsAroundStandoff and
  /// CalibrationOrchestratorNode::probeDirectionVisible.
  geometry_msgs::msg::Pose randomPoseNear(
    const geometry_msgs::msg::Pose & center_pose, double max_offset_m) const;

  /// Sends a single-waypoint ~/trace_path request (config_.planning_mode)
  /// and blocks for the response. Shared by both phases so
  /// runPolygonPhase/runRandomPhase/the return-to-center step don't
  /// duplicate it. Returns false if the service isn't available or the
  /// call fails.
  bool tracePathBlocking(const geometry_msgs::msg::Pose & target);

  /// Blocks (up to config_.sample_wait_timeout_sec) until a marker_pose
  /// message is received whose receipt time is after `after`, then
  /// returns it. Returns std::nullopt on timeout. This wait guarantees a
  /// sample reflects the arm's settled pose rather than whatever the most
  /// recently cached message happened to be.
  std::optional<geometry_msgs::msg::PoseStamped> waitForFreshMarkerPose(
    const rclcpp::Time & after);

  /// Gets exactly one marker sample for the current waypoint — the single
  /// point both runPolygonPhase/runRandomPhase's inner sampling loops
  /// call, so neither has to know which underlying mechanism actually
  /// produced the pose:
  /// - config_.hybrid_per_waypoint_enabled == false (default): identical
  ///   to the classical path — calls waitForFreshMarkerPose(after) against
  ///   the continuous marker_pose topic.
  /// - == true: brackets a single ~/detect_marker_once call (to
  ///   yolo_marker_bridge_node) with SIGCONT (before) / SIGSTOP (after)
  ///   via ~/signal_inference_server (on calibration_orchestrator_node —
  ///   see detect_marker_once_client_/signal_inference_server_client_ for
  ///   why this crosses a package boundary through a service rather than
  ///   a direct call). `after` is unused in this branch (each call is
  ///   inherently a fresh, on-demand detection) but kept in the signature
  ///   so both branches share one call site/signature in the sampling
  ///   loops. On success in this mode, also appends the returned crop
  ///   image + variant label to debug_grid_images_ for the end-of-run
  ///   combined grid (see accumulateDebugGridImage/saveDebugImageGrid).
  ///
  /// The ~/detect_marker_once call in the hybrid branch is bounded by
  /// config_.detect_call_timeout_sec. A timeout there returns
  /// std::nullopt, same as any other failed detection — the caller
  /// (runPolygonPhase/runRandomPhase) treats that uniformly via
  /// config_.min_samples_to_finish's discard-and-continue behavior.
  std::optional<geometry_msgs::msg::PoseStamped> sampleOnceAtCurrentWaypoint(
    const rclcpp::Time & after, const std::string & waypoint_label);

  /// Retries sampleOnceAtCurrentWaypoint up to
  /// config_.cal_ready_hybrid_marker_detection_retry times before giving
  /// up, taking a genuinely fresh frame each attempt (not a re-check of a
  /// stale result). Used by the center-pose block, runPolygonPhase, and
  /// runRandomPhase alike, so all three share one retry implementation.
  /// `after` is only meaningful for the first attempt in classical
  /// (non-hybrid) mode — each subsequent retry re-derives its own fresh
  /// boundary via get_clock()->now(), same as the per-sample
  /// sample_boundary advancement runPolygonPhase/runRandomPhase already
  /// do between samples_per_waypoint iterations.
  std::optional<geometry_msgs::msg::PoseStamped> sampleWithRetry(
    const rclcpp::Time & after, const std::string & waypoint_label);

  /// Sends SIGSTOP (stop=true) or SIGCONT (stop=false) to
  /// inference_server.py via ~/signal_inference_server on
  /// calibration_orchestrator_node — the same cross-package bridge
  /// sampleOnceAtCurrentWaypoint's per-waypoint bracketing uses (factored
  /// out here so executeCalibration's own end-of-run resume, which
  /// restarts continuous cup_holder/hole detection after a
  /// hybrid_per_waypoint_enabled run, doesn't duplicate this logic).
  /// Best-effort: logs (does not throw/abort) if the service isn't
  /// reachable.
  void signalInferenceServerViaOrchestrator(bool stop);

  /// Live (not cached) read of whether per-waypoint hybrid detection is
  /// currently active — see CalibrationBroadcasterConfig's comment on why
  /// this flag is deliberately not a struct field. Shared helper so every
  /// call site (sampleOnceAtCurrentWaypoint, and the isMarkerVisibleNow
  /// guards in runPolygonPhase/runRandomPhase) uses one get_parameter_or
  /// call instead of repeating it.
  bool isHybridPerWaypointEnabled() const;

  /// Assembles debug_grid_images_ into one labeled grid image (tiles
  /// arranged row-by-row via cv::hconcat/cv::vconcat, each waypoint's
  /// label burned in via cv::putText) and writes it once to the rosject's
  /// runtime log directory. Called once at the end of executeCalibration
  /// (success or failure — whatever was collected before a failure is
  /// still worth saving), only when debug_grid_images_ is non-empty (i.e.
  /// only in hybrid_per_waypoint_enabled mode, and only if at least one
  /// waypoint succeeded). Logs (not throws) on any write failure — this is
  /// a best-effort inspection artifact, not something that should fail
  /// the calibration run itself.
  void saveDebugImageGrid();

  /// Like waitForFreshMarkerPose, but only checks for visibility (doesn't
  /// need/return the pose itself) — used by the random phase's
  /// per-candidate visibility check, mirroring
  /// CalibrationOrchestratorNode::isMarkerVisibleAfter's polling pattern
  /// (a probe move to a genuinely-invisible position must time out
  /// gracefully rather than block on a condition variable).
  bool isMarkerVisibleNow(const rclcpp::Time & after);

  /// Chains one fresh marker_pose (camera_frame -> marker, from the
  /// detector) with the live known_chain_frame -> marker_frame TF into
  /// one sample of known_chain_frame -> camera, and appends it to
  /// collected_positions_/collected_orientations_. Returns false (logs
  /// the error) if the TF lookup fails.
  bool recordSample(const geometry_msgs::msg::PoseStamped & marker_pose);

  /// Early-stop check: called after every recordSample() success, in both
  /// phases. Computes the running position spread (max distance, in cm,
  /// of any collected sample's position from the arithmetic mean of all
  /// collected positions so far) and running orientation spread
  /// (max_spread_deg from averageQuaternions(collected_orientations_,
  /// averaging_method_) — safe to call mid-run, a pure function over
  /// whatever's collected so far). If both are within their respective
  /// tolerances (config_.position_spread_tolerance_cm/
  /// orientation_spread_tolerance_deg), increments
  /// stable_agreement_count_ (a running, non-consecutive count, not reset
  /// when a sample falls outside tolerance) and returns true once that
  /// counter reaches config_.stable_agreement_count. Returns false if
  /// fewer than 2 samples are collected yet (spread is meaningless with
  /// only 1 sample).
  bool stableAgreementReached();

  /// If config_.yaw_roll_clamp_enabled (see that field's doc comment for
  /// the motivation): decomposes every orientation in `orientations` to
  /// roll/pitch/yaw via tf2::Matrix3x3::getRPY(). Orientations here are
  /// known_chain_frame -> camera rotations, and base_link is
  /// REP-103-aligned (X forward, Y left, Z up), so getRPY's standard
  /// Z-Y-X intrinsic convention gives yaw = rotation about base_link's
  /// vertical axis, pitch = rotation about Y (tilt up/down), roll =
  /// rotation about X (in-plane image tilt). This differs from
  /// rotatedPoseNear()'s local end-effector-frame axis convention — do
  /// not conflate the two; this operates on an already-computed
  /// base_link-frame sample rotation, not a commanded arm pose.
  ///
  /// Computes a circular mean (atan2(mean(sin), mean(cos)), not a naive
  /// arithmetic mean, since angles wrap at +-pi) of yaw and of roll across
  /// every orientation in `orientations`, then returns a new vector where
  /// every sample's yaw and roll are replaced by those two run-wide means
  /// while each sample's own individually-measured pitch is left
  /// untouched. Returns `orientations` unchanged if
  /// config_.yaw_roll_clamp_enabled is false, or if `orientations` has
  /// fewer than 1 element.
  std::vector<tf2::Quaternion> clampYawRoll(
    const std::vector<tf2::Quaternion> & orientations) const;

  /// If config_.outlier_rejection_enabled: computes each collected
  /// sample's position deviation (cm, from the unfiltered arithmetic mean)
  /// and orientation deviation (degrees, from the unfiltered averaged
  /// quaternion), discards any sample exceeding either
  /// config_.outlier_position_threshold_cm or
  /// config_.outlier_orientation_threshold_deg, and returns the surviving
  /// indices. A clean run discards nothing. Logs how many samples were
  /// discarded, if any. When disabled, returns every index unfiltered.
  std::vector<size_t> rejectOutliers() const;

  /// Clustering-based position+orientation average, an alternative to the
  /// plain arithmetic mean/quaternion average used when
  /// use_clustering_average is true (read live via get_parameter(), not
  /// config_ — see CalibrationBroadcasterConfig's comment on why).
  /// Groups on both position and orientation because position-only
  /// clustering can still let an outlier orientation be included in the
  /// orientation average regardless of which position cluster it falls
  /// into, producing a visible orientation error even when position
  /// converges well.
  ///
  /// Algorithm: pairwise-distance clustering (not a fixed grid/histogram,
  /// which has an arbitrary origin/alignment problem that could split an
  /// otherwise-obvious cluster straddling a bucket boundary). Two samples
  /// (from `indices`, typically rejectOutliers()'s surviving set) are
  /// unioned into the same cluster only if both their straight-line
  /// position distance is <= position_bucket_size_cm and their angular
  /// orientation distance (angularDeviationDeg) is <=
  /// orientation_bucket_size_deg. O(n^2) pairwise comparison — fine at
  /// this data scale (dozens of samples). Returns the arithmetic-mean
  /// position and quaternion-averaged (kSumNormalize) orientation of just
  /// the largest cluster's members (ties broken by whichever cluster was
  /// formed first), plus that cluster's member indices (see
  /// ClusteredPose). Falls back to the plain mean/average of every index
  /// in `indices` if fewer than 2 samples are given.
  ClusteredPose computeClusteredPose(
    const std::vector<size_t> & indices, double position_bucket_size_cm,
    double orientation_bucket_size_deg) const;

  /// Averages collected_positions_/collected_orientations_ — first
  /// passing them through rejectOutliers() if
  /// config_.outlier_rejection_enabled, then computing the final position
  /// via either the plain arithmetic mean (default) or
  /// computeClusteredPosition() if the live (not cached)
  /// use_clustering_average parameter is true. Orientation always uses
  /// averaging_method_ (kSumNormalize by default) regardless of which
  /// position method is active — clustering is position-only for now.
  /// Broadcasts known_chain_frame -> the camera frame (from the most
  /// recent sample's header.frame_id) as a static TF, and completes
  /// goal_handle with the result (see Calibrate.action), including the
  /// is_high_confidence field (true if the post-rejection spread is
  /// within position_spread_tolerance_cm/orientation_spread_tolerance_deg
  /// — a soft, informational signal only; success is always true for a
  /// run that got this far, low confidence does not block the broadcast).
  /// Logs both the pre-rejection and post-rejection spread metrics when
  /// rejection actually discarded something. Clears both collected_
  /// vectors and resets stable_agreement_count_ for the next run.
  void finishCalibration(const std::shared_ptr<GoalHandleCalibrate> & goal_handle);

  CalibrationBroadcasterConfig config_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;
  /// Broadcasts each in-progress sample's candidate camera pose (see
  /// broadcastLatestSamplePose()) as known_chain_frame ->
  /// "camera_calibration_sample" — a plain (non-static) TransformBroadcaster
  /// deliberately, not static_broadcaster_ above: a static broadcast is
  /// meant to latch/persist as a fixed truth (correct for the final
  /// calibrated TF in finishCalibration()), but each sample here is
  /// transient and superseded by the next one. Publishing it as a plain
  /// (non-latched) TF means it simply stops updating once collection ends,
  /// rather than permanently overwriting the real calibrated frame with
  /// whatever the last sample happened to be.
  tf2_ros::TransformBroadcaster sample_tf_broadcaster_;
  /// Selected once at construction from config_'s priorities — see
  /// selectAveragingMethod.
  OrientationAveragingMethod averaging_method_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr marker_pose_sub_;
  rclcpp_action::Server<Calibrate>::SharedPtr calibrate_action_server_;
  rclcpp::Client<visual_calibration_msgs::srv::GetPolygonWaypoints>::SharedPtr
    get_polygon_waypoints_client_;
  rclcpp::Client<visual_calibration_msgs::srv::TracePath>::SharedPtr trace_path_client_;
  /// ~/detect_marker_once on yolo_marker_bridge_node — see
  /// config_.hybrid_per_waypoint_enabled's doc comment. Only ever called
  /// when that config field is true; unused otherwise (constructed
  /// regardless, matching get_polygon_waypoints_client_/trace_path_client_'s
  /// always-constructed convention).
  rclcpp::Client<visual_calibration_msgs::srv::DetectMarkerOnce>::SharedPtr
    detect_marker_once_client_;
  /// ~/signal_inference_server on calibration_orchestrator_node — see
  /// config_.hybrid_per_waypoint_enabled's doc comment (per-waypoint
  /// SIGCONT/SIGSTOP bracketing). Cross-package client: this node
  /// (aruco_perception) cannot call
  /// CalibrationOrchestratorNode::signalInferenceServer() directly (it's a
  /// private member of a different node/package) — this service is the
  /// intentional, thin cross-package bridge for it.
  rclcpp::Client<visual_calibration_msgs::srv::SignalInferenceServer>::SharedPtr
    signal_inference_server_client_;

  /// Guards latest_marker_pose_/latest_marker_pose_stamp_, notified by
  /// markerPoseCallback and waited on by waitForFreshMarkerPose.
  std::mutex sample_mutex_;
  std::condition_variable sample_cv_;
  geometry_msgs::msg::PoseStamped latest_marker_pose_;
  rclcpp::Time latest_marker_pose_stamp_;

  std::vector<geometry_msgs::msg::Vector3> collected_positions_;
  std::vector<tf2::Quaternion> collected_orientations_;

  /// One entry per successful hybrid_per_waypoint_enabled detection this
  /// run — the decoded, corner-annotated crop image (see aruco_pose.py's
  /// draw_detected_corners) plus the waypoint label ("waypoint 3
  /// (polygon)") and cascade variant ("gamma_0.7") as separate strings for
  /// saveDebugImageGrid's combined-grid capture. Cleared at the start of
  /// each ~/calibrate run (executeCalibration) alongside
  /// collected_positions_/collected_orientations_ — this is per-run state,
  /// not persisted across runs. Empty entirely when
  /// config_.hybrid_per_waypoint_enabled is false.
  struct DebugGridTile
  {
    cv::Mat image;
    std::string waypoint_label;
    std::string cascade_variant;
    /// Seconds the YOLO+cascade detection call itself took for this
    /// waypoint — see DetectMarkerOnce.srv's detect_time_s doc comment for
    /// exactly what this does/doesn't include. Drawn as a third label
    /// line in saveDebugImageGrid.
    double detect_time_s = 0.0;
  };
  std::vector<DebugGridTile> debug_grid_images_;
  /// The most recent sample's camera frame_id — carried through to the
  /// final broadcast's child_frame_id.
  geometry_msgs::msg::PoseStamped last_sample_;
  /// Running, non-consecutive count of samples found "in agreement" with
  /// the running average — see stableAgreementReached. Reset to 0 only in
  /// finishCalibration() (once per calibration run), not when a sample
  /// falls outside tolerance.
  int stable_agreement_count_ = 0;
  /// Random-offset generation (randomPoseNear) needs a seeded engine —
  /// member rather than a function-local static so it isn't shared/reset
  /// oddly across concurrent goals (executeCalibration runs one goal at a
  /// time per handleGoal's doc comment, but ordinary instance state is
  /// simpler here than reasoning about static init order).
  mutable std::mt19937 random_engine_{std::random_device{}()};
};

}  // namespace aruco_perception

#endif  // ARUCO_PERCEPTION__CALIBRATION_BROADCASTER_NODE_HPP_
