#ifndef ARUCO_PERCEPTION__CALIBRATION_BROADCASTER_NODE_HPP_
#define ARUCO_PERCEPTION__CALIBRATION_BROADCASTER_NODE_HPP_

#include <condition_variable>
#include <limits>
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
#include <tf2_ros/transform_broadcaster.h>
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

  // --- Orientation sweep phase (2026-07-29) ---
  /// When true, runs runOrientationSweepPhase() once after the polygon/
  /// random sampling is otherwise done (whether it stopped early or ran to
  /// completion), before finishCalibration(). Default true on real (the
  /// 37deg/14.9deg spread run that motivated this had zero orientation
  /// diversity beyond the polygon/random phases' position-only offsets —
  /// see randomPoseNear, which never varies orientation). Default false on
  /// sim (sim's ground-truth camera TF makes this extra probing largely
  /// redundant, and it costs real time/motion).
  bool orientation_sweep_enabled = false;
  /// Pitch/roll offset magnitude (degrees) used for all 4 sweep probes
  /// (pitch down, pitch up, roll left, roll right) — see
  /// runOrientationSweepPhase.
  double orientation_sweep_angle_deg = 5.0;

  // --- Outlier rejection (2026-07-29) ---
  /// When true, finishCalibration() discards any collected sample whose
  /// position or orientation deviation from the (unfiltered) mean exceeds
  /// outlier_position_threshold_cm / outlier_orientation_threshold_deg,
  /// before computing the final average. A sample is discarded if EITHER
  /// threshold is exceeded. Threshold-based (not fixed-worst-N): a clean
  /// run where every sample is already within both thresholds discards
  /// nothing.
  bool outlier_rejection_enabled = true;
  double outlier_position_threshold_cm = 2.0;
  double outlier_orientation_threshold_deg = 5.0;

  // --- Dual-sampling per waypoint (2026-07-29) ---
  /// Number of samples taken at each polygon/random-phase waypoint before
  /// moving to the next one (no additional move between them — same
  /// settled pose). Default 2: mitigates a single bad/missed detection
  /// being that waypoint's only data point. Samples from the same waypoint
  /// are pooled with every other sample (no separate same-waypoint
  /// agreement check) — outlier_rejection above is what sorts out any
  /// disagreement between them.
  int samples_per_waypoint = 2;

  // --- Clustering-based position+orientation averaging (2026-07-29) ---
  // clustering_bucket_size_cm/clustering_bucket_angle_deg are cached in
  // config_ like every other field above (tuning constants, fine to
  // require a restart to change). use_clustering_average is DELIBERATELY
  // NOT a field here — see finishCalibration()'s own comment for why it
  // must be read live via get_parameter() at the point of use, not cached
  // into this struct at construction (it's meant to be flippable from the
  // web UI's DevSpace drawer switch without a node restart, unlike every
  // other field in this struct, which IS restart-only by convention).
  //
  // Bucket/offset tolerance (cm) for treating two samples' POSITIONS as
  // "the same cluster" when use_clustering_average is true — see
  // computeClusteredPose()'s doc comment. Same default as
  // position_spread_tolerance_cm/outlier_position_threshold_cm (2.0cm) for
  // consistency, but an independently-tunable value, not a shared one —
  // confirmed via explicit instruction, not assumed.
  double clustering_bucket_size_cm = 2.0;
  // Angular tolerance (degrees) for treating two samples' ORIENTATIONS as
  // "the same cluster" (2026-07-29, added after position-only clustering
  // was found live to still leave a camera roll error — see
  // computeClusteredPose()'s doc comment). Both this AND
  // clustering_bucket_size_cm must hold for two samples to be grouped
  // together. Same default as orientation_spread_tolerance_deg/
  // outlier_orientation_threshold_deg (5.0deg) for consistency, but an
  // independently-tunable value.
  double clustering_bucket_angle_deg = 5.0;

  // --- Per-run yaw/roll clamp (2026-07-30) ---
  // The wall-mounted D415 (real) is physically fixed in yaw (cannot rotate
  // sideways without being unbolted/re-mounted) and effectively fixed in
  // roll (only minor, non-deliberate human-jostle variation) — pitch (tilt
  // up/down) is the only axis genuinely expected to vary. Confirmed as an
  // already-true structural fact for THIS project, not a new assumption:
  // sim's own ground-truth camera mount (base_link -> wrist_rgbd_camera_link,
  // from ur.urdf.xacro's sensor_r430 instantiation) has a fixed rpy=(0,
  // pi/3, pi/2) — roll and yaw are exact mechanical constants there too,
  // only pitch would ever legitimately change. aruco_detector_node does
  // zero corner/pose smoothing (confirmed 2026-07-30 investigation) — every
  // frame's raw ArUco-derived orientation flows straight into
  // collected_orientations_, so any yaw/roll variation observed across a
  // run's samples is corner-detection noise being misread as real
  // orientation change, not signal.
  //
  // When true, finishCalibration() computes a circular mean of yaw and of
  // roll across ALL collected samples THIS RUN (not a hardcoded constant —
  // real's true mount yaw/roll are unmeasured, unlike sim's URDF-given
  // values, so a per-run self-averaged reference is used instead), then
  // re-encodes every sample's orientation as (mean_roll, that sample's own
  // original pitch, mean_yaw) BEFORE rejectOutliers()/averaging runs on
  // them — see clampYawRoll()'s own doc comment for the full algorithm.
  // Pitch is deliberately left untouched per-sample: it is the one axis
  // that could carry real signal (e.g. a future orientation-sweep phase
  // deliberately varying pitch), so only the existing outlier-rejection/
  // averaging logic (unchanged) should ever filter it, not this clamp.
  //
  // Default false in BOTH sim and real yaml — this is an opt-in hypothesis
  // test, not a proven fix yet, and must not silently change today's
  // behavior. Restart-only (cached here like every other field in this
  // struct, unlike use_clustering_average above) — the clamp's own
  // per-run mean computation still runs fresh every finishCalibration()
  // call regardless; only the on/off switch itself is restart-only.
  bool yaw_roll_clamp_enabled = false;

  // --- forced yaw/roll test bypass (2026-08-03, real-only, throwaway) ---
  // Quick test hook, NOT a general feature: real's mount yaw/roll are now
  // physically bolted/measured (yaw=-180deg, roll=180deg), so this lets
  // clampYawRoll() use those known-correct constants directly for one test
  // run instead of re-deriving (possibly noisy) values from this run's own
  // samples via circular mean. NaN (default, unset) on either field means
  // "use today's existing circular-mean behavior for that axis" — set both
  // in degrees to bypass the mean computation entirely. Not wired into
  // sim's yaml at all: sim already has its own ground-truth mount angles
  // from the URDF and has no use for this.
  double yaw_roll_clamp_forced_yaw_deg = std::numeric_limits<double>::quiet_NaN();
  double yaw_roll_clamp_forced_roll_deg = std::numeric_limits<double>::quiet_NaN();
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

/// Result of computeClusteredPosition() (2026-07-29) — both position AND
/// orientation of the winning cluster's members, since clustering now
/// groups on BOTH (see that method's own doc comment for why: position-only
/// clustering still let outlier ORIENTATIONS drag the quaternion average,
/// producing a visible camera roll error even when position converged
/// well).
struct ClusteredPose
{
  geometry_msgs::msg::Vector3 position;
  tf2::Quaternion orientation;
  /// Local indices (into the `indices` vector passed to
  /// computeClusteredPosition(), NOT collected_positions_ directly) of the
  /// winning cluster's members — so the caller can compute a post-
  /// clustering spread/is_high_confidence check against exactly the
  /// samples that contributed to this result, not the full unfiltered set.
  std::vector<size_t> member_indices;
};

class CalibrationBroadcasterNode : public rclcpp::Node
{
public:
  using Calibrate = visual_calibration_msgs::action::Calibrate;
  using GoalHandleCalibrate = rclcpp_action::ServerGoalHandle<Calibrate>;

  CalibrationBroadcasterNode();

private:
  CalibrationBroadcasterConfig loadConfigFromParams() const;

  /// Total sample count a full (non-early-stopped) run will collect, used
  /// for feedback's samples_total field — 1 (center) +
  /// config_.num_samples * config_.samples_per_waypoint (polygon) +
  /// config_.random_phase_samples * config_.samples_per_waypoint (random)
  /// + (4 if config_.orientation_sweep_enabled, else 0). A single helper
  /// so this formula (2026-07-29: now scales with samples_per_waypoint and
  /// the sweep phase, not just "1 + num_samples + random_phase_samples")
  /// isn't duplicated at every call site that previously computed it
  /// inline.
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
  /// the original design used per-waypoint, but takes
  /// config_.samples_per_waypoint samples per settled pose (2026-07-29,
  /// default 2 — no additional move between them) rather than exactly
  /// one, before advancing to the next waypoint; early-stop is still
  /// checked after every individual sample, not just once per waypoint, so
  /// it can still trigger mid-waypoint on the first of the 2. Returns false (and sets
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
  /// discards before giving up as a hard failure. Like runPolygonPhase,
  /// takes config_.samples_per_waypoint samples per successfully-visible
  /// candidate (2026-07-29, default 2) before moving to the next
  /// candidate. Same out_result/stopped_early/return-value contract as
  /// runPolygonPhase.
  bool runRandomPhase(
    const std::shared_ptr<GoalHandleCalibrate> & goal_handle,
    const geometry_msgs::msg::Pose & center_pose,
    int samples_already_collected,
    std::shared_ptr<Calibrate::Result> & out_result,
    bool & stopped_early);

  /// Orientation sweep phase (2026-07-29): runs once, after the polygon/
  /// random sampling is otherwise done (early-stop or full count, either
  /// way), only when config_.orientation_sweep_enabled is true. Returns to
  /// cal_ready_pose, then probes 4 rotational offsets from its orientation
  /// — pitch down, pitch up, roll left, roll right, each
  /// config_.orientation_sweep_angle_deg degrees, each independently
  /// (not cumulative — always offset from cal_ready_pose's own
  /// orientation, not the previous probe's) — taking one sample per probe
  /// that successfully lands with the marker visible. A probe whose move
  /// fails or whose marker isn't visible is skipped (logged, not counted,
  /// not a hard failure) — same "a rotational extreme may lose the marker,
  /// that's expected" reasoning as runRandomPhase's invisible-marker
  /// handling, not a reason to abort an otherwise-successful run. Returns
  /// to cal_ready_pose again at the end, leaving the arm in a known pose
  /// before finishCalibration(). Samples are appended to the same
  /// collected_positions_/collected_orientations_ pool as every other
  /// sample — no separate pool or agreement check.
  void runOrientationSweepPhase(
    const std::shared_ptr<GoalHandleCalibrate> & goal_handle,
    const geometry_msgs::msg::Pose & cal_ready_pose,
    int samples_already_collected);

  /// Builds a geometry_msgs::msg::Pose from collected_positions_.back()/
  /// collected_orientations_.back() (2026-07-29) — the just-recorded
  /// sample, for populating Calibrate::Feedback::latest_sample_pose. Also
  /// broadcasts the SAME pose as a transient (non-static) TF frame,
  /// known_chain_frame -> "camera_calibration_sample" (a single frame that
  /// gets REPLACED every call, not accumulated — see sampleTfBroadcaster_'s
  /// doc comment for why a plain TransformBroadcaster, not
  /// StaticTransformBroadcaster, is used here), so RViz shows the live
  /// in-progress candidate pose updating sample by sample with zero
  /// frontend/message work required — this is the primary, always-on
  /// visualization; the web app's latest_sample_pose feedback field is an
  /// additional path for browser-side visualization, not a replacement.
  /// Must be called immediately after every recordSample() success, before
  /// building/publishing that sample's feedback message.
  geometry_msgs::msg::Pose broadcastLatestSamplePose();

  /// Builds a pose offset from `base_pose` by a pure rotation (pitch or
  /// roll) around `base_pose`'s own local axis, position unchanged — same
  /// tf2::Transform (base * offset) composition pattern as randomPoseNear,
  /// but offset here is rotation-only (translation zero) instead of
  /// randomPoseNear's translation-only offset. is_pitch selects which
  /// local axis the angle is applied around (pitch vs roll) — see
  /// runOrientationSweepPhase's call sites and this method's .cpp doc
  /// comment for which local axis maps to which, confirmed against
  /// trajectory_planner's end_effector_frame convention rather than
  /// assumed.
  geometry_msgs::msg::Pose rotatedPoseNear(
    const geometry_msgs::msg::Pose & base_pose, double angle_deg, bool is_pitch) const;

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

  /// If config_.yaw_roll_clamp_enabled (see that field's own doc comment
  /// for the full motivation): decomposes every orientation in
  /// `orientations` to roll/pitch/yaw via tf2::Matrix3x3::getRPY() —
  /// orientations here are known_chain_frame -> camera rotations (i.e.
  /// base_link -> camera on real), and base_link is REP-103-aligned
  /// (X forward, Y left, Z up, per the UR xacro's own comment), so
  /// getRPY's standard Z-Y-X intrinsic convention gives yaw = rotation
  /// about base_link's vertical (the axis a wall mount physically cannot
  /// rotate about), pitch = rotation about Y (tilt up/down, the one real
  /// DOF), roll = rotation about X (in-plane image tilt, also physically
  /// fixed by a rigid wall mount). This is DIFFERENT from
  /// rotatedPoseNear()'s local end-effector-frame axis convention (that
  /// function's own comment flags its axis mapping as unverified) — do
  /// not conflate the two; this operates on an already-computed
  /// base_link-frame sample rotation, not a commanded arm pose.
  ///
  /// Computes a CIRCULAR mean (atan2(mean(sin), mean(cos)), NOT a naive
  /// arithmetic mean — angles wrap at +-pi) of yaw and of roll across
  /// every orientation in `orientations`, then returns a new vector where
  /// every sample's yaw and roll are replaced by those two run-wide means
  /// while each sample's own individually-measured pitch is left
  /// untouched. Returns `orientations` unchanged if
  /// config_.yaw_roll_clamp_enabled is false, or if `orientations` has
  /// fewer than 1 element (mean is meaningless/trivial).
  std::vector<tf2::Quaternion> clampYawRoll(
    const std::vector<tf2::Quaternion> & orientations) const;

  /// If config_.outlier_rejection_enabled: computes each collected
  /// sample's position deviation (cm, from the unfiltered arithmetic mean)
  /// and orientation deviation (degrees, from the unfiltered averaged
  /// quaternion), discards any sample exceeding EITHER
  /// config_.outlier_position_threshold_cm or
  /// config_.outlier_orientation_threshold_deg, and returns the surviving
  /// indices. A clean run (nothing exceeds either threshold) discards
  /// nothing. Logs how many samples were discarded, if any. When disabled,
  /// returns every index unfiltered (a no-op pass-through).
  std::vector<size_t> rejectOutliers() const;

  /// Clustering-based position+orientation average (2026-07-29, extended
  /// to include orientation — an alternative to the plain arithmetic
  /// mean/quaternion average, used when use_clustering_average is true
  /// (read live via get_parameter(), NOT config_ — see
  /// CalibrationBroadcasterConfig's own comment on why). Motivated by a
  /// direct live observation: watching CalibratedCameraModel's in-progress
  /// sample meshes during a run showed many samples visibly clustering/
  /// overlapping near one location (with a small, consistent offset
  /// between them — never EXACTLY the same point) while a handful of
  /// outliers sat further away — a plain mean/quaternion-average lets
  /// those outliers drag the result off. Position-only clustering (the
  /// original 2026-07-29 version) still left orientation vulnerable to
  /// this — confirmed live: it produced a visible camera ROLL error even
  /// once position converged well, because an outlier sample's orientation
  /// could still be included in the orientation average regardless of
  /// which position cluster it fell into. This version requires BOTH
  /// position AND orientation agreement for two samples to be grouped
  /// together, closing that gap.
  ///
  /// Algorithm: pairwise-distance clustering (NOT a fixed grid/histogram —
  /// a fixed grid has an arbitrary origin/alignment problem that would
  /// split an otherwise-obvious cluster straddling a bucket boundary; the
  /// observed offsets are described as "close but not exact," i.e. fuzzy
  /// grouping, not points that will ever land in identical bins). Two
  /// samples (from `indices`, typically rejectOutliers()'s surviving set)
  /// are unioned into the same cluster ONLY if BOTH their straight-line
  /// position distance is <= position_bucket_size_cm AND their angular
  /// orientation distance (angularDeviationDeg) is <= orientation_bucket_size_deg.
  /// O(n^2) pairwise comparison — fine at this data scale (dozens of
  /// samples, not thousands), no spatial index needed. Returns the
  /// arithmetic-mean position and quaternion-averaged (kSumNormalize)
  /// orientation of just the LARGEST cluster's members (ties broken by
  /// whichever cluster was formed first), plus that cluster's member
  /// indices (see ClusteredPose's own doc comment for why). Falls back to
  /// the plain mean/average of every index in `indices` if fewer than 2
  /// samples are given (clustering is meaningless there).
  ClusteredPose computeClusteredPose(
    const std::vector<size_t> & indices, double position_bucket_size_cm,
    double orientation_bucket_size_deg) const;

  /// Averages collected_positions_/collected_orientations_ — first passing
  /// them through rejectOutliers() if config_.outlier_rejection_enabled,
  /// then computing the final position via EITHER the plain arithmetic
  /// mean (default) OR computeClusteredPosition() if the LIVE (not
  /// cached — see that field's own comment) use_clustering_average
  /// parameter is true. Orientation always uses averaging_method_
  /// (kSumNormalize today) regardless of which position method is
  /// active — clustering is position-only for now. Broadcasts
  /// known_chain_frame -> the camera frame (from the most recent sample's
  /// header.frame_id) as a static TF, and completes goal_handle with the
  /// result (see Calibrate.action), including the new is_high_confidence
  /// field (true if the POST-rejection spread is within
  /// position_spread_tolerance_cm/orientation_spread_tolerance_deg — a
  /// soft, informational signal only; success is ALWAYS true for a run
  /// that got this far, low confidence does not block the broadcast).
  /// Logs both the pre-rejection and post-rejection spread metrics when
  /// rejection actually discarded something, so the operator can see how
  /// much it helped. Clears both collected_ vectors AND resets
  /// stable_agreement_count_ for the next run.
  void finishCalibration(const std::shared_ptr<GoalHandleCalibrate> & goal_handle);

  CalibrationBroadcasterConfig config_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;
  /// Broadcasts each in-progress sample's candidate camera pose (2026-07-29,
  /// see broadcastLatestSamplePose()) as known_chain_frame ->
  /// "camera_calibration_sample" — a plain (non-static) TransformBroadcaster
  /// deliberately, NOT static_broadcaster_ above: a static broadcast is
  /// meant to latch/persist as a fixed truth (correct for the FINAL
  /// calibrated TF in finishCalibration()), but each sample here is a
  /// transient, superseded-by-the-next-one candidate — republishing it as
  /// a plain (non-latched) TF means it simply stops updating once
  /// collection ends, rather than permanently overwriting the real
  /// calibrated frame with whatever the last sample happened to be.
  tf2_ros::TransformBroadcaster sample_tf_broadcaster_;
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
