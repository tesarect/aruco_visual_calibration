#include "aruco_perception/calibration_broadcaster_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <future>
#include <map>
#include <stdexcept>
#include <thread>
#include <tuple>
#include <vector>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace aruco_perception
{

CalibrationBroadcasterNode::CalibrationBroadcasterNode()
: Node(
    "calibration_broadcaster_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
  config_(loadConfigFromParams()),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_),
  static_broadcaster_(this),
  sample_tf_broadcaster_(this),
  averaging_method_(
    selectAveragingMethod(
      config_.orientation_sum_normalize_priority, config_.orientation_markley_priority))
{
  marker_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    config_.marker_pose_topic, 10,
    std::bind(&CalibrationBroadcasterNode::markerPoseCallback, this, std::placeholders::_1));

  calibrate_action_server_ = rclcpp_action::create_server<Calibrate>(
    this,
    "~/calibrate",
    std::bind(
      &CalibrationBroadcasterNode::handleGoal, this, std::placeholders::_1,
      std::placeholders::_2),
    std::bind(&CalibrationBroadcasterNode::handleCancel, this, std::placeholders::_1),
    std::bind(&CalibrationBroadcasterNode::handleAccepted, this, std::placeholders::_1));

  get_polygon_waypoints_client_ =
    create_client<visual_calibration_msgs::srv::GetPolygonWaypoints>(
    "/trajectory_planner/get_polygon_waypoints");
  trace_path_client_ = create_client<visual_calibration_msgs::srv::TracePath>(
    "/trajectory_planner/trace_path");

  // Only ever called when isHybridPerWaypointEnabled() is true — see that
  // method's own doc comment. Constructed unconditionally anyway,
  // matching get_polygon_waypoints_client_/trace_path_client_'s own
  // always-constructed convention (cheap, no harm sitting unused).
  detect_marker_once_client_ = create_client<visual_calibration_msgs::srv::DetectMarkerOnce>(
    "/yolo_marker_bridge_node/detect_marker_once");
  signal_inference_server_client_ =
    create_client<visual_calibration_msgs::srv::SignalInferenceServer>(
    "/calibration_orchestrator_node/signal_inference_server");

  // Which of kSumNormalize/kMarkley selectAveragingMethod() actually picked
  // (2026-08-03) — added after discovering neither the raw priority values
  // nor the resulting method were logged anywhere, making a test run's log
  // alone insufficient to confirm which averaging method it actually
  // exercised (e.g. a "markley_priority-1" test run silently falling back
  // to kSumNormalize if orientation_sum_normalize_priority wasn't also set
  // to 0 — see selectAveragingMethod's own tie-break: sum_normalize wins
  // whenever its priority is <= markley's).
  RCLCPP_INFO(
    get_logger(),
    "Orientation averaging method selected: %s (orientation_sum_normalize_priority: %d, "
    "orientation_markley_priority: %d)",
    averaging_method_ == OrientationAveragingMethod::kMarkley ? "kMarkley" : "kSumNormalize",
    config_.orientation_sum_normalize_priority, config_.orientation_markley_priority);

  RCLCPP_INFO(
    get_logger(), "calibration_broadcaster_node ready (known_chain_frame: '%s', marker_frame: "
    "'%s', num_samples: %d) — send a ~/calibrate action goal to begin",
    config_.known_chain_frame.c_str(), config_.marker_frame.c_str(), config_.num_samples);
}

void CalibrationBroadcasterNode::markerPoseCallback(
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(sample_mutex_);
  latest_marker_pose_ = *msg;
  latest_marker_pose_stamp_ = get_clock()->now();
  sample_cv_.notify_all();
}

rclcpp_action::GoalResponse CalibrationBroadcasterNode::handleGoal(
  const rclcpp_action::GoalUUID &/*uuid*/,
  std::shared_ptr<const Calibrate::Goal>/*goal*/)
{
  RCLCPP_INFO(get_logger(), "Received ~/calibrate goal");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CalibrationBroadcasterNode::handleCancel(
  const std::shared_ptr<GoalHandleCalibrate>/*goal_handle*/)
{
  RCLCPP_INFO(get_logger(), "Cancelling ~/calibrate goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void CalibrationBroadcasterNode::handleAccepted(
  const std::shared_ptr<GoalHandleCalibrate> goal_handle)
{
  // rclcpp_action requires this callback to return quickly — the actual
  // orchestration (service calls + waiting) runs on its own thread.
  std::thread{
    std::bind(&CalibrationBroadcasterNode::executeCalibration, this, std::placeholders::_1),
    goal_handle}.detach();
}

// Guarantees two things run on EVERY exit path out of executeCalibration
// (2026-08-04) — that function has several early `return`s on failure
// (service unavailable, sample timeout, phase failure, cancellation) in
// addition to its normal success path ending in finishCalibration();
// inserting cleanup calls before every individual return would be fragile
// (easy to miss one on a future edit) — a destructor-based guard makes it
// structurally impossible to skip instead:
// 1. saveDebugImageGrid() — per hybrid_per_waypoint_enabled's design
//    ("whatever was collected before a failure is still worth saving").
// 2. Resume inference_server.py if THIS run itself was in hybrid mode
//    (isHybridPerWaypointEnabled(), captured at construction time — see
//    that flag's own comment on why a captured snapshot, not a live
//    re-check, is used here) — per-waypoint bracketing always leaves the
//    model SIGSTOPped after its last sample; without this, continuous
//    cup_holder/hole detection (yolo_marker_bridge_node's own
//    image_callback, independent of this hybrid mechanism) would silently
//    stop working after every hybrid calibration run, since it keeps
//    calling into a paused process. Not gated behind
//    isHybridPerWaypointEnabled() again here (a live re-check) — the web
//    switch could have flipped mid-run; this guard's job is to clean up
//    whatever state THIS run itself left behind, not to reflect
//    whatever the CURRENT live setting happens to be by the time it runs.
//
// Declared directly in namespace aruco_perception (NOT an anonymous
// namespace, unlike this file's other local helpers) so its friend
// declaration inside CalibrationBroadcasterNode (see that class's own
// comment) actually names the SAME type — an anonymous-namespace
// definition here would be a distinct type from the one the friend
// declaration forward-declares in the enclosing namespace, silently
// failing to grant access (confirmed via a live compile error).
struct EndOfRunCleanupGuard
{
  EndOfRunCleanupGuard(CalibrationBroadcasterNode * node, bool was_hybrid_this_run)
  : node_(node), was_hybrid_this_run_(was_hybrid_this_run) {}
  ~EndOfRunCleanupGuard()
  {
    node_->saveDebugImageGrid();
    if (was_hybrid_this_run_) {
      node_->signalInferenceServerViaOrchestrator(false);  // SIGCONT
    }
  }
  CalibrationBroadcasterNode * node_;
  bool was_hybrid_this_run_;
};

void CalibrationBroadcasterNode::executeCalibration(
  const std::shared_ptr<GoalHandleCalibrate> goal_handle)
{
  // Captured ONCE here, at the very start — see EndOfRunCleanupGuard's own
  // doc comment for why this snapshot (not a live re-check in the
  // destructor) is what decides whether to resume inference_server.py at
  // the end.
  EndOfRunCleanupGuard end_of_run_cleanup_guard(this, isHybridPerWaypointEnabled());

  collected_positions_.clear();
  collected_orientations_.clear();
  debug_grid_images_.clear();
  stable_agreement_count_ = 0;

  // Logged once, up front, for this run (2026-08-04) — isHybridPerWaypointEnabled()
  // is checked many times over a run (once per sample), which would be
  // noisy to log every time; this single line at the start gives a clear,
  // easy-to-find answer to "was this particular run in hybrid mode" when
  // inspecting a captured log, without needing to infer it indirectly
  // from whether sampleOnceAtCurrentWaypoint's own per-sample timing
  // lines happen to appear.
  RCLCPP_INFO(
    get_logger(), "executeCalibration starting: hybrid_per_waypoint_enabled=%s, "
    "min_samples_to_finish=%d, samples_per_waypoint=%d",
    isHybridPerWaypointEnabled() ? "true" : "false", config_.min_samples_to_finish,
    config_.samples_per_waypoint);

  if (!get_polygon_waypoints_client_->wait_for_service(std::chrono::seconds(5))) {
    auto result = std::make_shared<Calibrate::Result>();
    result->success = false;
    result->message = "trajectory_planner's ~/get_polygon_waypoints service not available";
    goal_handle->abort(result);
    RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
    return;
  }

  auto waypoints_request =
    std::make_shared<visual_calibration_msgs::srv::GetPolygonWaypoints::Request>();
  auto waypoints_future = get_polygon_waypoints_client_->async_send_request(waypoints_request);
  const auto waypoints_response = waypoints_future.get();

  if (!waypoints_response->success || waypoints_response->waypoints.empty()) {
    auto result = std::make_shared<Calibrate::Result>();
    result->success = false;
    result->message = "Could not fetch polygon waypoints: " + waypoints_response->message;
    goal_handle->abort(result);
    RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
    return;
  }

  const std::vector<geometry_msgs::msg::Pose> waypoints(
    waypoints_response->waypoints.begin(), waypoints_response->waypoints.end());
  const geometry_msgs::msg::Pose center_pose = waypoints_response->center_pose;
  RCLCPP_INFO(get_logger(), "Fetched %zu polygon waypoints", waypoints.size());

  if (!trace_path_client_->wait_for_service(std::chrono::seconds(5))) {
    auto result = std::make_shared<Calibrate::Result>();
    result->success = false;
    result->message = "trajectory_planner's ~/trace_path service not available";
    goal_handle->abort(result);
    RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
    return;
  }

  // Sample once at the center pose itself, right after it's known — the
  // arm is already there (center_pose IS trajectory_planner's own current
  // pose, per polygonWaypointsAroundStandoff's 2026-07-22 redesign), so
  // this needs no additional move, just an immediate marker_pose wait +
  // record before the polygon phase's first corner move begins. Counted
  // toward the same running total as every other sample.
  {
    const rclcpp::Time now = get_clock()->now();
    const std::optional<geometry_msgs::msg::PoseStamped> center_marker_pose =
      sampleOnceAtCurrentWaypoint(now, "center pose");

    if (!center_marker_pose.has_value()) {
      auto result = std::make_shared<Calibrate::Result>();
      result->success = false;
      result->message = "Timed out waiting for a fresh marker_pose at the center pose "
        "(is the marker still in view?)";
      goal_handle->abort(result);
      RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
      return;
    }

    if (!recordSample(*center_marker_pose)) {
      auto result = std::make_shared<Calibrate::Result>();
      result->success = false;
      result->message = "Could not record the center-pose sample (TF lookup failed, see log)";
      goal_handle->abort(result);
      RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
      return;
    }

    const int total_samples = totalSamplesTarget();
    RCLCPP_INFO(get_logger(), "Collected sample 1/%d (center pose)", total_samples);

    auto feedback = std::make_shared<Calibrate::Feedback>();
    feedback->samples_collected = static_cast<uint32_t>(collected_positions_.size());
    feedback->samples_total = static_cast<uint32_t>(total_samples);
    feedback->latest_sample_pose = broadcastLatestSamplePose();
    goal_handle->publish_feedback(feedback);
  }

  std::shared_ptr<Calibrate::Result> phase_result;
  bool stopped_early = stableAgreementReached();

  if (!stopped_early && !runPolygonPhase(goal_handle, waypoints, phase_result, stopped_early)) {
    if (phase_result) {
      goal_handle->abort(phase_result);
      RCLCPP_ERROR(get_logger(), "%s", phase_result->message.c_str());
    }
    // else: cancellation already handled (goal_handle->canceled) inside
    // runPolygonPhase itself.
    return;
  }

  if (!stopped_early) {
    if (!runRandomPhase(
        goal_handle, center_pose, static_cast<int>(collected_positions_.size()), phase_result,
        stopped_early))
    {
      if (phase_result) {
        goal_handle->abort(phase_result);
        RCLCPP_ERROR(get_logger(), "%s", phase_result->message.c_str());
      }
      return;
    }
  }

  // Orientation sweep phase (2026-07-29): runs once here, regardless of
  // WHICH prior phase last ran or whether it stopped early or ran to
  // completion — neither polygon nor random phase varies orientation at
  // all (randomPoseNear only offsets X/Y/Z, see its own doc comment), so
  // this is the only source of orientation-diverse samples. Gated by
  // config_.orientation_sweep_enabled (default off in sim, on in real —
  // see CalibrationBroadcasterConfig's doc comment). Soft-fails per probe
  // (skips, doesn't abort the run) — see runOrientationSweepPhase's own
  // doc comment — so no phase_result/goal_handle->abort branch is needed
  // here the way the two phases above need one.
  if (config_.orientation_sweep_enabled) {
    runOrientationSweepPhase(
      goal_handle, center_pose, static_cast<int>(collected_positions_.size()));
  }

  // Discard-and-continue's actual pass/fail gate (2026-08-04) — see
  // config_.min_samples_to_finish's own doc comment. Only checked when
  // opted in (> 0); at the default 0, every prior s==0 failure already
  // hard-aborted via runPolygonPhase/runRandomPhase's own early return, so
  // execution would never reach here with too few samples in the first
  // place — this check is a no-op in that case, not redundant with it.
  if (config_.min_samples_to_finish > 0 &&
    static_cast<int>(collected_positions_.size()) < config_.min_samples_to_finish)
  {
    auto result = std::make_shared<Calibrate::Result>();
    result->success = false;
    result->message = "Only " + std::to_string(collected_positions_.size()) +
      " sample(s) collected, below min_samples_to_finish (" +
      std::to_string(config_.min_samples_to_finish) + ") — too few waypoints succeeded to "
      "produce a meaningful calibration";
    goal_handle->abort(result);
    RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
    return;
  }

  finishCalibration(goal_handle);
}

bool CalibrationBroadcasterNode::runPolygonPhase(
  const std::shared_ptr<GoalHandleCalibrate> & goal_handle,
  const std::vector<geometry_msgs::msg::Pose> & waypoints,
  std::shared_ptr<Calibrate::Result> & out_result,
  bool & stopped_early)
{
  stopped_early = false;
  const int total_samples = totalSamplesTarget();

  for (int i = 0; i < config_.num_samples; ++i) {
    if (goal_handle->is_canceling()) {
      auto result = std::make_shared<Calibrate::Result>();
      result->success = false;
      result->message = "Calibration cancelled";
      goal_handle->canceled(result);
      RCLCPP_INFO(get_logger(), "Calibration cancelled");
      out_result = nullptr;
      return false;
    }

    // Cycle through the polygon's corners if num_samples exceeds their
    // count, so a longer run still spreads across the same set of poses
    // rather than failing or stopping early.
    const geometry_msgs::msg::Pose & target = waypoints[i % waypoints.size()];

    // Captured BEFORE the move, not after tracePathBlocking() returns —
    // waitForFreshMarkerPose's whole purpose is rejecting any marker_pose
    // that could have arrived during the move, so the timestamp boundary
    // must predate the move starting, not just predate it settling (a
    // regression risk introduced by extracting tracePathBlocking() as a
    // shared helper during the 2026-07-22 two-phase redesign — fixed
    // here; see error-mitigation.md #19 for why this matters).
    const rclcpp::Time before_move = get_clock()->now();
    if (!tracePathBlocking(target)) {
      out_result = std::make_shared<Calibrate::Result>();
      out_result->success = false;
      out_result->message = "~/trace_path failed for sample " + std::to_string(i + 1);
      return false;
    }

    // Take config_.samples_per_waypoint samples at this SAME settled pose
    // (2026-07-29 — no additional move between them) before advancing to
    // the next waypoint, mitigating a single bad/missed detection being
    // this waypoint's only data point. Both/all go into the same
    // collected_ pool as every other sample — no same-waypoint agreement
    // check, rejectOutliers() sorts out any disagreement between them
    // later.
    //
    // sample_boundary tracks the freshness cutoff for THIS waypoint's next
    // wait — starts at before_move (s==0, same as before this loop
    // existed), then advances to "now" after each successful wait. Fixed
    // 2026-07-29: originally every s reused the same before_move boundary
    // unconditionally, which meant s==1 (and beyond) could — and reliably
    // did, live — pass the ">before_move" check against the SAME still-
    // cached message s==0 already consumed, since nothing in that
    // condition required the message to be NEW relative to the previous
    // sample, only new relative to before_move. Confirmed live: every
    // waypoint's 2 samples showed byte-identical deviation, meaning both
    // were the same underlying detection counted twice, not 2 independent
    // measurements — silently doubling every sample's weight (good or
    // bad) rather than adding real new information.
    rclcpp::Time sample_boundary = before_move;
    for (int s = 0; s < config_.samples_per_waypoint; ++s) {
      const std::optional<geometry_msgs::msg::PoseStamped> marker_pose =
        sampleOnceAtCurrentWaypoint(
          sample_boundary, "waypoint " + std::to_string(i + 1) + " (polygon)");

      if (!marker_pose.has_value()) {
        if (s > 0 || config_.min_samples_to_finish > 0) {
          // Soft-fail (2026-07-29 for s>0; extended 2026-08-04 to s==0
          // too, when config_.min_samples_to_finish is opted in — see
          // that field's own doc comment for the full rationale). The
          // marker can genuinely drop out of view/fail detection at any
          // single waypoint — for s>0 that was already known to be
          // non-fatal (samples from every prior waypoint are unaffected);
          // for s==0 (this waypoint's ONLY attempt failing, the common
          // case under hybrid_per_waypoint_enabled with
          // samples_per_waypoint=1) it's now equally non-fatal whenever
          // the opt-in is active, deferring the actual pass/fail decision
          // to executeCalibration's own end-of-run min_samples_to_finish
          // check instead of aborting immediately on the first miss.
          RCLCPP_WARN(
            get_logger(), "Polygon-phase sample %d: marker lost after %d/%d samples at this "
            "waypoint — keeping what was collected, moving to the next waypoint",
            i + 1, s, config_.samples_per_waypoint);

          // Web-UI-visible status (2026-08-04) — samples_collected/
          // samples_total unchanged (this event didn't add a sample), but
          // current_status carries the skip so an operator watching the
          // left panel sees WHY progress isn't advancing on this
          // waypoint, instead of the UI looking silently stuck. See
          // Calibrate.action's own doc comment on this field.
          {
            auto skip_feedback = std::make_shared<Calibrate::Feedback>();
            skip_feedback->samples_collected = static_cast<uint32_t>(collected_positions_.size());
            skip_feedback->samples_total = static_cast<uint32_t>(total_samples);
            skip_feedback->current_status = "Waypoint " + std::to_string(i + 1) +
              " (polygon): sample skipped — marker not found";
            goal_handle->publish_feedback(skip_feedback);
          }
          break;
        }
        out_result = std::make_shared<Calibrate::Result>();
        out_result->success = false;
        out_result->message = "Timed out waiting for a fresh marker_pose for sample " +
          std::to_string(i + 1) + " (is the marker still in view?)";
        return false;
      }

      // Advance the boundary to now (receipt-time domain, matching
      // latest_marker_pose_stamp_'s own get_clock()->now() assignment in
      // markerPoseCallback — NOT marker_pose->header.stamp, which is the
      // sensor's own publish time, a different clock/value) — the NEXT
      // sample at this waypoint (if any) must wait for a message that
      // arrives after THIS one was consumed, not just after the move
      // settled.
      sample_boundary = get_clock()->now();

      if (!recordSample(*marker_pose)) {
        out_result = std::make_shared<Calibrate::Result>();
        out_result->success = false;
        out_result->message = "Could not record sample " + std::to_string(i + 1) +
          " (TF lookup failed, see log)";
        return false;
      }

      RCLCPP_INFO(
        get_logger(), "Collected sample %zu/%d (polygon phase)", collected_positions_.size(),
        total_samples);

      auto feedback = std::make_shared<Calibrate::Feedback>();
      feedback->samples_collected = static_cast<uint32_t>(collected_positions_.size());
      feedback->samples_total = static_cast<uint32_t>(total_samples);
      feedback->latest_sample_pose = broadcastLatestSamplePose();
      goal_handle->publish_feedback(feedback);

      if (stableAgreementReached()) {
        RCLCPP_INFO(
          get_logger(), "Early-stop: agreement reached after %zu samples (polygon phase)",
          collected_positions_.size());
        stopped_early = true;
        return true;
      }
    }
  }

  return true;
}

bool CalibrationBroadcasterNode::runRandomPhase(
  const std::shared_ptr<GoalHandleCalibrate> & goal_handle,
  const geometry_msgs::msg::Pose & center_pose,
  int samples_already_collected,
  std::shared_ptr<Calibrate::Result> & out_result,
  bool & stopped_early)
{
  stopped_early = false;
  const int total_samples = totalSamplesTarget();
  int consecutive_failures = 0;

  for (int i = 0; i < config_.random_phase_samples; ) {
    if (goal_handle->is_canceling()) {
      auto result = std::make_shared<Calibrate::Result>();
      result->success = false;
      result->message = "Calibration cancelled";
      goal_handle->canceled(result);
      RCLCPP_INFO(get_logger(), "Calibration cancelled");
      out_result = nullptr;
      return false;
    }

    const geometry_msgs::msg::Pose candidate =
      randomPoseNear(center_pose, config_.random_phase_max_offset_m);

    // Captured BEFORE the move — see runPolygonPhase's identical comment
    // on why this must predate the move starting, not just its settling.
    const rclcpp::Time before_move = get_clock()->now();
    if (!tracePathBlocking(candidate)) {
      // A failed move (e.g. planAndExecuteCartesian refusing an
      // incomplete straight-line path — see trajectory_planner.cpp's
      // cartesian_min_fraction check) is treated the same as an
      // invisible-marker candidate: discarded, not counted, retried with
      // a new random candidate, bounded by the same consecutive-failure
      // cap (2026-07-23 — a random offset can point in any direction, so
      // occasionally landing on one direction's Cartesian-path limit is
      // expected, not a reason to abort an otherwise-successful run). No
      // return-to-center needed here — planAndExecuteCartesian refuses
      // BEFORE calling execute() on an incomplete path, so the arm never
      // actually moved; it's already still at the last good pose.
      ++consecutive_failures;
      RCLCPP_INFO(
        get_logger(), "Random-phase candidate's move failed (attempt %d/%d consecutive) — "
        "trying a new candidate", consecutive_failures,
        config_.random_phase_max_consecutive_failures);

      if (consecutive_failures >= config_.random_phase_max_consecutive_failures) {
        out_result = std::make_shared<Calibrate::Result>();
        out_result->success = false;
        out_result->message = "Random phase gave up after " +
          std::to_string(consecutive_failures) + " consecutive failed/invisible candidates";
        return false;
      }
      continue;
    }

    // isMarkerVisibleNow polls the continuous marker_pose topic —
    // meaningless under hybrid_per_waypoint_enabled (nothing publishes
    // that topic in this mode; see runOrientationSweepPhase's identical
    // fix/comment). Skipped there; sampleOnceAtCurrentWaypoint's own
    // std::nullopt return in the loop below already covers "not visible."
    if (!isHybridPerWaypointEnabled() && !isMarkerVisibleNow(before_move)) {
      // Discarded, not counted — return to center immediately (no point
      // probing further out when not visible at all here) and try a new
      // candidate.
      ++consecutive_failures;
      RCLCPP_INFO(
        get_logger(), "Random-phase candidate not visible (attempt %d/%d consecutive) — "
        "returning to center", consecutive_failures, config_.random_phase_max_consecutive_failures);

      if (consecutive_failures >= config_.random_phase_max_consecutive_failures) {
        out_result = std::make_shared<Calibrate::Result>();
        out_result->success = false;
        out_result->message = "Random phase gave up after " +
          std::to_string(consecutive_failures) + " consecutive failed/invisible candidates";
        return false;
      }

      if (!tracePathBlocking(center_pose)) {
        out_result = std::make_shared<Calibrate::Result>();
        out_result->success = false;
        out_result->message = "Could not return to center pose after an invisible random candidate";
        return false;
      }
      continue;
    }

    consecutive_failures = 0;

    // Take config_.samples_per_waypoint samples at this SAME visible
    // candidate pose (2026-07-29 — no additional move between them, same
    // reasoning as runPolygonPhase's identical change) before moving to
    // the next random candidate. sample_boundary advances after each
    // successful wait — see runPolygonPhase's identical variable/fix for
    // why reusing before_move unconditionally across all samples_per_
    // waypoint iterations was a bug (every sample after the first could
    // return the SAME still-cached message, confirmed live).
    rclcpp::Time sample_boundary = before_move;
    for (int s = 0; s < config_.samples_per_waypoint; ++s) {
      const std::optional<geometry_msgs::msg::PoseStamped> marker_pose =
        sampleOnceAtCurrentWaypoint(
          sample_boundary,
          "waypoint " + std::to_string(samples_already_collected + i + 1) + " (random)");
      if (!marker_pose.has_value()) {
        if (s > 0 || config_.min_samples_to_finish > 0) {
          // Soft-fail (2026-07-29 for s>0; extended 2026-08-04 to s==0
          // too, when config_.min_samples_to_finish is opted in — see
          // that field's own doc comment, and runPolygonPhase's identical
          // extension, for the full rationale). isMarkerVisibleNow() above
          // only checks visibility ONCE, before this loop starts — it does
          // not guarantee the marker stays visible/detectable across every
          // sample. Confirmed live, repeatedly: the marker can genuinely
          // drop out of view between sample s=0 and s=1 at a random-phase
          // candidate (position offset from cal_ready can be enough to
          // lose it), and previously this hard-failed the ENTIRE
          // calibration run over losing just the 2nd of 2 samples at ONE
          // candidate — discarding 21+ good samples already collected.
          RCLCPP_WARN(
            get_logger(), "Random-phase sample %d: marker lost after %d/%d samples at this "
            "candidate — keeping what was collected, moving to the next candidate",
            samples_already_collected + i + 1, s, config_.samples_per_waypoint);

          // Web-UI-visible status (2026-08-04) — see runPolygonPhase's
          // identical addition/comment for the full rationale.
          {
            auto skip_feedback = std::make_shared<Calibrate::Feedback>();
            skip_feedback->samples_collected = static_cast<uint32_t>(collected_positions_.size());
            skip_feedback->samples_total = static_cast<uint32_t>(total_samples);
            skip_feedback->current_status = "Waypoint " +
              std::to_string(samples_already_collected + i + 1) +
              " (random): sample skipped — marker not found";
            goal_handle->publish_feedback(skip_feedback);
          }
          break;
        }
        out_result = std::make_shared<Calibrate::Result>();
        out_result->success = false;
        out_result->message =
          "Timed out waiting for a fresh marker_pose for random-phase sample " +
          std::to_string(samples_already_collected + i + 1);
        return false;
      }
      sample_boundary = get_clock()->now();

      if (!recordSample(*marker_pose)) {
        out_result = std::make_shared<Calibrate::Result>();
        out_result->success = false;
        out_result->message = "Could not record random-phase sample " +
          std::to_string(samples_already_collected + i + 1) + " (TF lookup failed, see log)";
        return false;
      }

      RCLCPP_INFO(
        get_logger(), "Collected sample %zu/%d (random phase)", collected_positions_.size(),
        total_samples);

      auto feedback = std::make_shared<Calibrate::Feedback>();
      feedback->samples_collected = static_cast<uint32_t>(collected_positions_.size());
      feedback->samples_total = static_cast<uint32_t>(total_samples);
      feedback->latest_sample_pose = broadcastLatestSamplePose();
      goal_handle->publish_feedback(feedback);

      if (stableAgreementReached()) {
        RCLCPP_INFO(
          get_logger(), "Early-stop: agreement reached after %zu samples (random phase)",
          collected_positions_.size());
        stopped_early = true;
        return true;
      }
    }

    ++i;
  }

  return true;
}

void CalibrationBroadcasterNode::runOrientationSweepPhase(
  const std::shared_ptr<GoalHandleCalibrate> & goal_handle,
  const geometry_msgs::msg::Pose & cal_ready_pose,
  int samples_already_collected)
{
  const int total_samples = totalSamplesTarget();
  int samples_collected_this_phase = 0;

  // (angle sign, is_pitch, label) for the 4 probes — each independently
  // offset from cal_ready_pose's own orientation (not cumulative from the
  // previous probe), matching runOrientationSweepPhase's doc comment.
  const std::array<std::tuple<double, bool, const char *>, 4> probes = {{
      {-config_.orientation_sweep_angle_deg, true, "pitch down"},
      {config_.orientation_sweep_angle_deg, true, "pitch up"},
      {-config_.orientation_sweep_angle_deg, false, "roll left"},
      {config_.orientation_sweep_angle_deg, false, "roll right"},
    }};

  for (const auto & [angle_deg, is_pitch, label] : probes) {
    if (goal_handle->is_canceling()) {
      // Mid-sweep cancellation: leave sample collection where it is —
      // executeCalibration's caller (finishCalibration) will run on
      // whatever was collected so far, same as the polygon/random phases'
      // own cancellation handling elsewhere aborts the goal outright, but
      // this phase is explicitly best-effort/soft-fail (see this method's
      // header doc comment), so a cancellation here just stops taking
      // further sweep samples rather than discarding the run.
      RCLCPP_INFO(get_logger(), "Orientation sweep phase: cancellation requested, stopping early");
      return;
    }

    const geometry_msgs::msg::Pose target = rotatedPoseNear(cal_ready_pose, angle_deg, is_pitch);

    const rclcpp::Time before_move = get_clock()->now();
    if (!tracePathBlocking(target)) {
      RCLCPP_INFO(
        get_logger(), "Orientation sweep: '%s' probe move failed — skipping this probe "
        "(not a hard failure, see runOrientationSweepPhase's doc comment)", label);
      continue;
    }

    // isMarkerVisibleNow polls the continuous marker_pose topic, same as
    // waitForFreshMarkerPose below — meaningless under
    // hybrid_per_waypoint_enabled (nothing publishes that topic in this
    // mode), so this pre-check is skipped there; sampleOnceAtCurrentWaypoint
    // below already returns std::nullopt on a failed/absent detection,
    // which this loop already treats identically to "not visible."
    if (!isHybridPerWaypointEnabled() && !isMarkerVisibleNow(before_move)) {
      RCLCPP_INFO(
        get_logger(), "Orientation sweep: marker not visible at '%s' probe — skipping this "
        "probe", label);
      continue;
    }

    const std::optional<geometry_msgs::msg::PoseStamped> marker_pose =
      sampleOnceAtCurrentWaypoint(before_move, std::string("orientation sweep: ") + label);
    if (!marker_pose.has_value()) {
      RCLCPP_INFO(
        get_logger(), "Orientation sweep: timed out waiting for a fresh marker_pose at '%s' "
        "probe — skipping this probe", label);
      continue;
    }

    if (!recordSample(*marker_pose)) {
      RCLCPP_WARN(
        get_logger(), "Orientation sweep: could not record '%s' probe's sample (TF lookup "
        "failed, see log) — skipping this probe", label);
      continue;
    }

    ++samples_collected_this_phase;
    RCLCPP_INFO(
      get_logger(), "Collected sample %zu/%d (orientation sweep, '%s')",
      collected_positions_.size(), total_samples, label);

    auto feedback = std::make_shared<Calibrate::Feedback>();
    feedback->samples_collected = static_cast<uint32_t>(collected_positions_.size());
    feedback->samples_total = static_cast<uint32_t>(total_samples);
    feedback->latest_sample_pose = broadcastLatestSamplePose();
    goal_handle->publish_feedback(feedback);
  }

  // Return to cal_ready regardless of how many probes succeeded, leaving
  // the arm in a known pose before finishCalibration() — a soft-fail here
  // (logged, not fatal) since the calibration itself has already collected
  // everything it needs; only the arm's final resting pose is at stake.
  if (!tracePathBlocking(cal_ready_pose)) {
    RCLCPP_WARN(
      get_logger(), "Orientation sweep phase: failed to return to cal_ready after sweeping "
      "(non-fatal — calibration will still finish and broadcast normally)");
  }

  RCLCPP_INFO(
    get_logger(), "Orientation sweep phase complete: %d/4 probes collected (samples_already_"
    "collected was %d)", samples_collected_this_phase, samples_already_collected);
}

geometry_msgs::msg::Pose CalibrationBroadcasterNode::randomPoseNear(
  const geometry_msgs::msg::Pose & center_pose, double max_offset_m) const
{
  // Uniform offset within a cube of side 2*max_offset_m, re-rolled until
  // its straight-line distance from center is within max_offset_m (a
  // simple rejection sampler — cheap at this scale, no closed-form
  // uniform-sphere sampling needed).
  std::uniform_real_distribution<double> axis_dist(-max_offset_m, max_offset_m);

  double dx = 0.0;
  double dy = 0.0;
  double dz = 0.0;
  do {
    dx = axis_dist(random_engine_);
    dy = axis_dist(random_engine_);
    dz = axis_dist(random_engine_);
  } while (std::sqrt(dx * dx + dy * dy + dz * dz) > max_offset_m);

  tf2::Transform center;
  tf2::fromMsg(center_pose, center);
  const tf2::Transform offset(tf2::Quaternion::getIdentity(), tf2::Vector3(dx, dy, dz));
  const tf2::Transform result = center * offset;

  geometry_msgs::msg::Pose result_pose;
  result_pose.position.x = result.getOrigin().x();
  result_pose.position.y = result.getOrigin().y();
  result_pose.position.z = result.getOrigin().z();
  result_pose.orientation = tf2::toMsg(result.getRotation());
  return result_pose;
}

geometry_msgs::msg::Pose CalibrationBroadcasterNode::rotatedPoseNear(
  const geometry_msgs::msg::Pose & base_pose, double angle_deg, bool is_pitch) const
{
  // Same tf2::Transform (base * offset) composition pattern as
  // randomPoseNear, but offset here is rotation-only (translation zero)
  // around base_pose's own LOCAL axis, applied on the right so it's
  // expressed in base_pose's frame, not world frame — position stays
  // exactly at base_pose's origin (a pure orientation probe, not a
  // combined position+orientation move).
  //
  // Axis convention: is_pitch rotates around the local Y axis (tf2::Vector3(0,1,0)),
  // roll around the local X axis (tf2::Vector3(1,0,0)) — the standard
  // aerospace/robotics convention (pitch = rotation about Y, roll =
  // rotation about X, when Z is the forward/approach axis). NOT verified
  // against config_.end_effector_frame's ("robotiq_85_base_link") actual
  // URDF-defined joint axis orientation — that would require walking the
  // full parent-joint <origin rpy=...> chain, not done here. If the first
  // live sweep-phase test moves the wrist in the direction labeled "roll"
  // when you expected "pitch" (or vice versa), swap is_pitch's axis
  // mapping below (X<->Y) — this is the one thing about this function
  // that's a documented assumption, not a confirmed fact.
  const tf2::Vector3 axis = is_pitch ? tf2::Vector3(0, 1, 0) : tf2::Vector3(1, 0, 0);
  const tf2::Quaternion offset_rotation(axis, angle_deg * M_PI / 180.0);

  tf2::Transform base;
  tf2::fromMsg(base_pose, base);
  const tf2::Transform offset(offset_rotation, tf2::Vector3(0, 0, 0));
  const tf2::Transform result = base * offset;

  geometry_msgs::msg::Pose result_pose;
  result_pose.position.x = result.getOrigin().x();
  result_pose.position.y = result.getOrigin().y();
  result_pose.position.z = result.getOrigin().z();
  result_pose.orientation = tf2::toMsg(result.getRotation());
  return result_pose;
}

bool CalibrationBroadcasterNode::tracePathBlocking(const geometry_msgs::msg::Pose & target)
{
  auto trace_request = std::make_shared<visual_calibration_msgs::srv::TracePath::Request>();
  trace_request->waypoints = {target};
  trace_request->planning_mode = config_.planning_mode;

  auto trace_future = trace_path_client_->async_send_request(trace_request);
  const auto trace_response = trace_future.get();
  return trace_response->success;
}

std::optional<geometry_msgs::msg::PoseStamped> CalibrationBroadcasterNode::waitForFreshMarkerPose(
  const rclcpp::Time & after)
{
  std::unique_lock<std::mutex> lock(sample_mutex_);
  const bool got_fresh_sample = sample_cv_.wait_for(
    lock, std::chrono::duration<double>(config_.sample_wait_timeout_sec),
    [this, &after]() {
      return latest_marker_pose_stamp_.nanoseconds() > 0 && latest_marker_pose_stamp_ > after;
    });

  if (!got_fresh_sample) {
    return std::nullopt;
  }
  return latest_marker_pose_;
}

namespace
{
/// Standard base64 -> raw bytes decode (2026-08-04) — no existing
/// dependency in this package already provides one (confirmed via grep),
/// and the payload here (one JPEG crop per waypoint, DetectMarkerOnce.srv's
/// cascade_image_b64 field) doesn't justify pulling in a new library
/// dependency for it. Inverse of Python's base64.b64encode, used
/// symmetrically on inference_server.py's _encode_image_b64 side.
std::vector<uchar> decodeBase64(const std::string & encoded)
{
  static const std::string kChars =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::vector<int> decode_table(256, -1);
  for (size_t i = 0; i < kChars.size(); ++i) {
    decode_table[static_cast<unsigned char>(kChars[i])] = static_cast<int>(i);
  }

  std::vector<uchar> result;
  result.reserve(encoded.size() / 4 * 3);

  int val = 0, bits = -8;
  for (unsigned char c : encoded) {
    if (c == '=' || decode_table[c] == -1) {
      if (c == '=') {break;}
      continue;  // skip whitespace/newlines, if any
    }
    val = (val << 6) + decode_table[c];
    bits += 6;
    if (bits >= 0) {
      result.push_back(static_cast<uchar>((val >> bits) & 0xFF));
      bits -= 8;
    }
  }
  return result;
}
}  // namespace

bool CalibrationBroadcasterNode::isHybridPerWaypointEnabled() const
{
  bool enabled = false;
  get_parameter_or("hybrid_per_waypoint_enabled", enabled, false);
  return enabled;
}

void CalibrationBroadcasterNode::signalInferenceServerViaOrchestrator(bool stop)
{
  // Best-effort: logs (doesn't abort the caller) if the orchestrator's
  // service isn't reachable — a missing/late SIGCONT just means
  // inference_server.py takes its normal startup-from-stopped time on the
  // next actual request, not a hard failure by itself.
  if (!signal_inference_server_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(
      get_logger(),
      "signalInferenceServerViaOrchestrator(%s): ~/signal_inference_server not "
      "reachable — is calibration_orchestrator_node running?", stop ? "SIGSTOP" : "SIGCONT");
    return;
  }
  auto request = std::make_shared<visual_calibration_msgs::srv::SignalInferenceServer::Request>();
  request->stop = stop;
  signal_inference_server_client_->async_send_request(request).wait();
}

std::optional<geometry_msgs::msg::PoseStamped>
CalibrationBroadcasterNode::sampleOnceAtCurrentWaypoint(
  const rclcpp::Time & after, const std::string & waypoint_label)
{
  if (!isHybridPerWaypointEnabled()) {
    return waitForFreshMarkerPose(after);
  }

  // Per-waypoint SIGCONT/SIGSTOP bracketing (2026-08-04) — see
  // signalInferenceServerViaOrchestrator's own doc comment.
  auto send_signal = [this](bool stop) {signalInferenceServerViaOrchestrator(stop);};

  // Timing (2026-08-04) — no timeout enforced yet on the detect call below
  // (future.get() blocks unconditionally), per explicit request: measure
  // real per-sample cost first, THEN pick a bounded timeout value/param on
  // a later change once actual numbers are known, rather than guessing a
  // number now. Logged unconditionally (not just on failure) so a full
  // run's log gives a real distribution to look at, not just outliers.
  const rclcpp::Time call_start = get_clock()->now();

  send_signal(false);  // SIGCONT — resume inference_server.py for this one call
  const rclcpp::Time after_sigcont = get_clock()->now();

  if (!detect_marker_once_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(
      get_logger(),
      "sampleOnceAtCurrentWaypoint: ~/detect_marker_once not reachable — is "
      "yolo_marker_bridge_node running?");
    send_signal(true);  // SIGSTOP — don't leave the model running idle on failure
    return std::nullopt;
  }

  auto request = std::make_shared<visual_calibration_msgs::srv::DetectMarkerOnce::Request>();
  auto future = detect_marker_once_client_->async_send_request(request);

  // Bounded wait (2026-08-04, was previously an unbounded future.get() —
  // see config_.detect_call_timeout_sec's own doc comment) — real timing
  // logs across several test runs showed 4.30s-10.51s per call; this
  // param defaults well above that observed range plus a safety margin,
  // not tuned to the tightest possible value.
  const auto wait_status = future.wait_for(
    std::chrono::duration<double>(config_.detect_call_timeout_sec));
  const rclcpp::Time after_detect = get_clock()->now();

  send_signal(true);  // SIGSTOP — done with this waypoint's single call
  const rclcpp::Time after_sigstop = get_clock()->now();

  if (wait_status != std::future_status::ready) {
    RCLCPP_ERROR(
      get_logger(),
      "sampleOnceAtCurrentWaypoint (%s): ~/detect_marker_once call exceeded "
      "detect_call_timeout_sec (%.1fs) — treating this waypoint as failed "
      "(sigcont=%.2fs, gave up after=%.2fs, sigstop=%.2fs)",
      waypoint_label.c_str(), config_.detect_call_timeout_sec,
      (after_sigcont - call_start).seconds(), (after_detect - after_sigcont).seconds(),
      (after_sigstop - after_detect).seconds());
    // NOTE: the underlying yolo_marker_bridge_node-side request is NOT
    // cancelled by giving up on the future here — it may still complete
    // and its result will simply be discarded when this rclcpp::Client
    // eventually receives it. Accepted trade-off: cancelling an in-flight
    // ROS2 service call cleanly is nontrivial, and this is already the
    // rare/timeout case, not the common path.
    return std::nullopt;
  }

  const auto response = future.get();

  RCLCPP_INFO(
    get_logger(),
    "sampleOnceAtCurrentWaypoint (%s) timing: sigcont=%.2fs detect=%.2fs sigstop=%.2fs "
    "total=%.2fs",
    waypoint_label.c_str(),
    (after_sigcont - call_start).seconds(),
    (after_detect - after_sigcont).seconds(),
    (after_sigstop - after_detect).seconds(),
    (after_sigstop - call_start).seconds());

  if (!response->success) {
    RCLCPP_WARN(
      get_logger(), "sampleOnceAtCurrentWaypoint (%s): %s",
      waypoint_label.c_str(), response->message.c_str());
    return std::nullopt;
  }

  // Accumulate for the end-of-run combined debug grid (see
  // saveDebugImageGrid) — best-effort: a decode failure here doesn't
  // invalidate the pose itself, just skips that one tile.
  if (!response->cascade_image_b64.empty()) {
    const std::vector<uchar> jpeg_bytes = decodeBase64(response->cascade_image_b64);
    const cv::Mat decoded = cv::imdecode(jpeg_bytes, cv::IMREAD_COLOR);
    if (!decoded.empty()) {
      const std::string label = waypoint_label + ": " + response->cascade_variant_used;
      debug_grid_images_.emplace_back(decoded, label);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "sampleOnceAtCurrentWaypoint (%s): failed to decode cascade_image_b64 — "
        "skipping this tile in the debug grid", waypoint_label.c_str());
    }
  }

  return response->marker_pose;
}

void CalibrationBroadcasterNode::saveDebugImageGrid()
{
  if (debug_grid_images_.empty()) {
    // No-op whenever hybrid_per_waypoint_enabled is false (default), or if
    // it's true but zero waypoints succeeded this run — nothing worth
    // writing either way.
    return;
  }

  // Normalize every tile to a common FIXED width AND height (2026-08-04,
  // fixed a live crash) — the original version only forced a common width
  // and let each tile's height scale proportionally to its own source
  // image's aspect ratio, which meant real tiles could legitimately end up
  // with DIFFERENT heights from each other (not just from the blank
  // padding tile) — hconcat requires every image in a row to share the
  // exact same height, and any row mixing differently-scaled tiles crashed
  // with "Assertion failed: src[i].rows == src[0].rows" (confirmed live).
  // Fix: letterbox every tile into an identical kTileWidth x kTileHeight
  // canvas (preserving each source image's own aspect ratio via uniform
  // scale-to-fit, centered, black bars on the short axis) rather than
  // trusting proportional resize to ever coincidentally match.
  const int kTileWidth = debug_grid_images_.front().first.cols;
  const int kTileHeight = debug_grid_images_.front().first.rows;
  const int kTilesPerRow = 4;
  const cv::Scalar kLabelBgColor(0, 0, 0);
  const cv::Scalar kLabelTextColor(0, 255, 0);
  const int kLabelStripHeight = 22;

  std::vector<cv::Mat> tiles;
  tiles.reserve(debug_grid_images_.size());
  for (const auto & [image, label] : debug_grid_images_) {
    // Scale-to-fit within kTileWidth x kTileHeight, preserving aspect
    // ratio, then center on a black canvas of EXACTLY that size — every
    // tile this loop produces has identical dimensions, regardless of its
    // source image's own size/aspect ratio.
    const double scale = std::min(
      static_cast<double>(kTileWidth) / image.cols,
      static_cast<double>(kTileHeight) / image.rows);
    cv::Mat scaled;
    cv::resize(
      image, scaled,
      cv::Size(std::max(1, static_cast<int>(image.cols * scale)),
        std::max(1, static_cast<int>(image.rows * scale))));

    cv::Mat canvas(kTileHeight, kTileWidth, image.type(), kLabelBgColor);
    const int x_offset = (kTileWidth - scaled.cols) / 2;
    const int y_offset = (kTileHeight - scaled.rows) / 2;
    scaled.copyTo(canvas(cv::Rect(x_offset, y_offset, scaled.cols, scaled.rows)));

    // Label strip burned in at the bottom of each tile (2026-08-04) — self-
    // contained labeling per user's explicit request ("just for visual
    // inspection, or even to show during presentation"), so the single
    // saved grid image needs no separate caption file to be meaningful.
    cv::Mat labeled(kTileHeight + kLabelStripHeight, kTileWidth, canvas.type(), kLabelBgColor);
    canvas.copyTo(labeled(cv::Rect(0, 0, kTileWidth, kTileHeight)));
    cv::putText(
      labeled, label, cv::Point(4, kTileHeight + kLabelStripHeight - 6),
      cv::FONT_HERSHEY_SIMPLEX, 0.4, kLabelTextColor, 1, cv::LINE_AA);
    tiles.push_back(labeled);
  }

  // Pad the tile count up to a full multiple of kTilesPerRow with blank
  // tiles of matching size — hconcat below requires every image in a row
  // to share the same height, and a ragged final row would otherwise need
  // special-casing. Safe now: every real tile above is already the exact
  // same (kTileWidth, kTileHeight + kLabelStripHeight) size, so this blank
  // tile matches all of them, not just the first.
  const cv::Mat blank_tile(tiles.front().rows, tiles.front().cols, tiles.front().type(), kLabelBgColor);
  while (tiles.size() % kTilesPerRow != 0) {
    tiles.push_back(blank_tile);
  }

  std::vector<cv::Mat> rows;
  for (size_t row_start = 0; row_start < tiles.size(); row_start += kTilesPerRow) {
    cv::Mat row;
    cv::hconcat(
      std::vector<cv::Mat>(tiles.begin() + row_start, tiles.begin() + row_start + kTilesPerRow),
      row);
    rows.push_back(row);
  }

  cv::Mat grid;
  cv::vconcat(rows, grid);

  // /home/user/ros2_ws/log/tmux/real (2026-08-04) — the rosject's own
  // runtime log directory, per explicit user confirmation (NOT this local
  // checkout's own _errors/real/ archive convention, a separate thing).
  const std::string kOutputDir = "/home/user/ros2_ws/log/tmux/real";
  const std::string output_path = kOutputDir + "/hybrid_per_waypoint_debug_grid.png";
  if (!cv::imwrite(output_path, grid)) {
    RCLCPP_ERROR(
      get_logger(),
      "saveDebugImageGrid: failed to write '%s' (%zu tiles) — does the directory exist?",
      output_path.c_str(), debug_grid_images_.size());
    return;
  }

  RCLCPP_INFO(
    get_logger(), "saveDebugImageGrid: wrote %zu waypoint tiles to '%s'",
    debug_grid_images_.size(), output_path.c_str());
}

bool CalibrationBroadcasterNode::isMarkerVisibleNow(const rclcpp::Time & after)
{
  // Polls rather than blocking on the condition variable (unlike
  // waitForFreshMarkerPose) — a random-phase candidate at a position
  // where the marker is genuinely out of view will never produce a fresh
  // message at all, so this has to time out gracefully, not hang. Same
  // pattern as CalibrationOrchestratorNode::isMarkerVisibleAfter.
  const rclcpp::Time deadline =
    get_clock()->now() + rclcpp::Duration::from_seconds(config_.sample_wait_timeout_sec);

  while (get_clock()->now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(sample_mutex_);
      if (latest_marker_pose_stamp_.nanoseconds() > 0 && latest_marker_pose_stamp_ > after) {
        return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return false;
}

bool CalibrationBroadcasterNode::recordSample(
  const geometry_msgs::msg::PoseStamped & marker_pose)
{
  // marker_pose is camera_frame -> marker_frame (camera's own frame_id,
  // marker's pose within it). Invert to get marker_frame -> camera_frame,
  // then chain with the known known_chain_frame -> marker_frame TF to get
  // one sample of known_chain_frame -> camera_frame.
  tf2::Transform camera_to_marker;
  tf2::fromMsg(marker_pose.pose, camera_to_marker);
  const tf2::Transform marker_to_camera = camera_to_marker.inverse();

  geometry_msgs::msg::TransformStamped known_to_marker_tf;
  try {
    known_to_marker_tf = tf_buffer_.lookupTransform(
      config_.known_chain_frame, config_.marker_frame, tf2::TimePointZero,
      tf2::durationFromSec(1.0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(
      get_logger(), "Could not look up '%s' -> '%s': %s",
      config_.known_chain_frame.c_str(), config_.marker_frame.c_str(), ex.what());
    return false;
  }

  tf2::Transform known_to_marker;
  tf2::fromMsg(known_to_marker_tf.transform, known_to_marker);
  const tf2::Transform known_to_camera = known_to_marker * marker_to_camera;

  geometry_msgs::msg::Vector3 sample_position;
  sample_position.x = known_to_camera.getOrigin().x();
  sample_position.y = known_to_camera.getOrigin().y();
  sample_position.z = known_to_camera.getOrigin().z();
  collected_positions_.push_back(sample_position);
  collected_orientations_.push_back(known_to_camera.getRotation());

  last_sample_.header = marker_pose.header;
  last_sample_.pose.position.x = sample_position.x;
  last_sample_.pose.position.y = sample_position.y;
  last_sample_.pose.position.z = sample_position.z;
  last_sample_.pose.orientation = tf2::toMsg(known_to_camera.getRotation());

  return true;
}

geometry_msgs::msg::Pose CalibrationBroadcasterNode::broadcastLatestSamplePose()
{
  // last_sample_.pose already holds exactly the sample recordSample() just
  // computed (position + orientation) — no need to re-derive it from
  // collected_positions_.back()/collected_orientations_.back().
  geometry_msgs::msg::TransformStamped sample_tf;
  sample_tf.header.stamp = get_clock()->now();
  sample_tf.header.frame_id = config_.known_chain_frame;
  // Fixed name, not per-sample-numbered — this frame is meant to be
  // watched live in RViz as ONE thing that updates, not accumulate one
  // frame per sample (that's what the web app's per-sample pose array,
  // fed by latest_sample_pose, is for instead).
  sample_tf.child_frame_id = "camera_calibration_sample";
  sample_tf.transform.translation.x = last_sample_.pose.position.x;
  sample_tf.transform.translation.y = last_sample_.pose.position.y;
  sample_tf.transform.translation.z = last_sample_.pose.position.z;
  sample_tf.transform.rotation = last_sample_.pose.orientation;

  sample_tf_broadcaster_.sendTransform(sample_tf);

  return last_sample_.pose;
}

int CalibrationBroadcasterNode::totalSamplesTarget() const
{
  const int polygon_and_random =
    (config_.num_samples + config_.random_phase_samples) * config_.samples_per_waypoint;
  const int sweep = config_.orientation_sweep_enabled ? 4 : 0;
  return 1 + polygon_and_random + sweep;
}

bool CalibrationBroadcasterNode::stableAgreementReached()
{
  const size_t count = collected_positions_.size();
  if (count < 2) {
    // Spread is meaningless with only 0/1 samples collected.
    return false;
  }

  geometry_msgs::msg::Vector3 mean_position;
  for (const geometry_msgs::msg::Vector3 & position : collected_positions_) {
    mean_position.x += position.x;
    mean_position.y += position.y;
    mean_position.z += position.z;
  }
  mean_position.x /= static_cast<double>(count);
  mean_position.y /= static_cast<double>(count);
  mean_position.z /= static_cast<double>(count);

  double max_position_spread_m = 0.0;
  for (const geometry_msgs::msg::Vector3 & position : collected_positions_) {
    const double dx = position.x - mean_position.x;
    const double dy = position.y - mean_position.y;
    const double dz = position.z - mean_position.z;
    max_position_spread_m = std::max(max_position_spread_m, std::sqrt(dx * dx + dy * dy + dz * dz));
  }
  const double max_position_spread_cm = max_position_spread_m * 100.0;

  const OrientationAveragingResult orientation_result =
    averageQuaternions(collected_orientations_, averaging_method_);

  const bool within_tolerance =
    max_position_spread_cm <= config_.position_spread_tolerance_cm &&
    orientation_result.max_spread_deg <= config_.orientation_spread_tolerance_deg;

  if (within_tolerance) {
    ++stable_agreement_count_;
    RCLCPP_INFO(
      get_logger(), "Early-stop check: sample within tolerance (position spread %.2fcm, "
      "orientation spread %.2fdeg) — agreement count %d/%d",
      max_position_spread_cm, orientation_result.max_spread_deg, stable_agreement_count_,
      config_.stable_agreement_count);
  }
  // Deliberately NOT reset on a single out-of-tolerance sample — see this
  // method's doc comment (a running, non-consecutive count).

  return stable_agreement_count_ >= config_.stable_agreement_count;
}

std::vector<tf2::Quaternion> CalibrationBroadcasterNode::clampYawRoll(
  const std::vector<tf2::Quaternion> & orientations) const
{
  if (!config_.yaw_roll_clamp_enabled || orientations.size() < 1) {
    return orientations;
  }

  // Circular mean — a naive arithmetic mean of angles breaks near the
  // +-pi wraparound (e.g. mean of +179deg and -179deg should be 180deg,
  // not 0deg). atan2(mean(sin), mean(cos)) is the standard fix.
  double yaw_sin_sum = 0.0, yaw_cos_sum = 0.0;
  double roll_sin_sum = 0.0, roll_cos_sum = 0.0;

  std::vector<std::array<double, 3>> rpy_per_sample;
  rpy_per_sample.reserve(orientations.size());

  for (const tf2::Quaternion & q : orientations) {
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    rpy_per_sample.push_back({roll, pitch, yaw});

    yaw_sin_sum += std::sin(yaw);
    yaw_cos_sum += std::cos(yaw);
    roll_sin_sum += std::sin(roll);
    roll_cos_sum += std::cos(roll);
  }

  // Forced yaw/roll test bypass (2026-08-03, real-only — see
  // CalibrationBroadcasterConfig::yaw_roll_clamp_forced_yaw_deg's own doc
  // comment) — NaN (default/unset) means "use the circular mean computed
  // above for that axis," same as before this bypass existed. A non-NaN
  // value skips straight to the known, physically-measured mount constant
  // instead, so this run's own (possibly noisy) samples never factor into
  // that axis at all.
  const bool yaw_forced = !std::isnan(config_.yaw_roll_clamp_forced_yaw_deg);
  const bool roll_forced = !std::isnan(config_.yaw_roll_clamp_forced_roll_deg);
  const double mean_yaw = yaw_forced ?
    config_.yaw_roll_clamp_forced_yaw_deg * M_PI / 180.0 :
    std::atan2(yaw_sin_sum, yaw_cos_sum);
  const double mean_roll = roll_forced ?
    config_.yaw_roll_clamp_forced_roll_deg * M_PI / 180.0 :
    std::atan2(roll_sin_sum, roll_cos_sum);

  RCLCPP_INFO(
    get_logger(),
    "clampYawRoll: run-wide mean yaw=%.2fdeg%s, mean roll=%.2fdeg%s (computed from %zu samples) "
    "— every sample's yaw/roll replaced with these, pitch left as individually measured",
    mean_yaw * 180.0 / M_PI, yaw_forced ? " (forced)" : "",
    mean_roll * 180.0 / M_PI, roll_forced ? " (forced)" : "", orientations.size());

  std::vector<tf2::Quaternion> clamped;
  clamped.reserve(orientations.size());
  for (size_t i = 0; i < orientations.size(); ++i) {
    const double original_pitch = rpy_per_sample[i][1];
    tf2::Quaternion clamped_q;
    clamped_q.setRPY(mean_roll, original_pitch, mean_yaw);
    clamped.push_back(clamped_q);

    RCLCPP_DEBUG(
      get_logger(),
      "clampYawRoll: sample %zu pre-clamp rpy=(%.2f, %.2f, %.2f)deg -> post-clamp "
      "rpy=(%.2f, %.2f, %.2f)deg",
      i, rpy_per_sample[i][0] * 180.0 / M_PI, rpy_per_sample[i][1] * 180.0 / M_PI,
      rpy_per_sample[i][2] * 180.0 / M_PI, mean_roll * 180.0 / M_PI,
      original_pitch * 180.0 / M_PI, mean_yaw * 180.0 / M_PI);
  }

  return clamped;
}

std::vector<size_t> CalibrationBroadcasterNode::rejectOutliers() const
{
  const size_t count = collected_positions_.size();
  std::vector<size_t> all_indices(count);
  for (size_t i = 0; i < count; ++i) {
    all_indices[i] = i;
  }

  if (!config_.outlier_rejection_enabled || count < 3) {
    // Rejecting anything from fewer than 3 samples risks discarding half
    // (or all) the data over noise that can't be distinguished from a
    // genuinely valid 2-sample disagreement — not attempted.
    return all_indices;
  }

  geometry_msgs::msg::Vector3 mean_position;
  for (const geometry_msgs::msg::Vector3 & position : collected_positions_) {
    mean_position.x += position.x;
    mean_position.y += position.y;
    mean_position.z += position.z;
  }
  mean_position.x /= static_cast<double>(count);
  mean_position.y /= static_cast<double>(count);
  mean_position.z /= static_cast<double>(count);

  const OrientationAveragingResult unfiltered_orientation =
    averageQuaternions(collected_orientations_, averaging_method_);

  std::vector<size_t> kept_indices;
  for (size_t i = 0; i < count; ++i) {
    const geometry_msgs::msg::Vector3 & position = collected_positions_[i];
    const double dx = position.x - mean_position.x;
    const double dy = position.y - mean_position.y;
    const double dz = position.z - mean_position.z;
    const double position_deviation_cm = std::sqrt(dx * dx + dy * dy + dz * dz) * 100.0;

    const double orientation_deviation_deg =
      angularDeviationDeg(collected_orientations_[i], unfiltered_orientation.averaged);

    const bool is_outlier =
      position_deviation_cm > config_.outlier_position_threshold_cm ||
      orientation_deviation_deg > config_.outlier_orientation_threshold_deg;

    if (is_outlier) {
      RCLCPP_INFO(
        get_logger(), "Outlier rejection: discarding sample %zu (position deviation %.2fcm, "
        "orientation deviation %.2fdeg)", i, position_deviation_cm, orientation_deviation_deg);
    } else {
      kept_indices.push_back(i);
    }
  }

  if (kept_indices.size() < 2) {
    // Rejecting down to 0/1 samples would make the final average
    // meaningless (or leave nothing to average at all) — safer to keep
    // everything than to broadcast a TF derived from a single sample.
    RCLCPP_WARN(
      get_logger(), "Outlier rejection would leave only %zu sample(s) — keeping all %zu "
      "samples instead (rejection skipped this run)", kept_indices.size(), count);
    return all_indices;
  }

  return kept_indices;
}

ClusteredPose CalibrationBroadcasterNode::computeClusteredPose(
  const std::vector<size_t> & indices, double position_bucket_size_cm,
  double orientation_bucket_size_deg) const
{
  auto plainAverage = [this](const std::vector<size_t> & idx) {
      geometry_msgs::msg::Vector3 mean;
      std::vector<tf2::Quaternion> orientations;
      orientations.reserve(idx.size());
      for (const size_t i : idx) {
        mean.x += collected_positions_[i].x;
        mean.y += collected_positions_[i].y;
        mean.z += collected_positions_[i].z;
        orientations.push_back(collected_orientations_[i]);
      }
      const double n = static_cast<double>(idx.size());
      mean.x /= n;
      mean.y /= n;
      mean.z /= n;

      ClusteredPose result;
      result.position = mean;
      result.orientation = orientations.empty() ?
        tf2::Quaternion::getIdentity() :
        averageQuaternions(orientations, averaging_method_).averaged;
      result.member_indices = idx;
      return result;
    };

  if (indices.size() < 2) {
    return plainAverage(indices);
  }

  // Union-find over `indices` (not the raw 0..collected_positions_.size()
  // range — indices may already be a filtered subset from rejectOutliers()).
  // parent[k] indexes into `indices` itself (local indices 0..indices.size()-1),
  // not into collected_positions_ directly.
  std::vector<size_t> parent(indices.size());
  for (size_t k = 0; k < indices.size(); ++k) {
    parent[k] = k;
  }

  std::function<size_t(size_t)> find = [&](size_t k) {
      while (parent[k] != k) {
        parent[k] = parent[parent[k]];  // path compression
        k = parent[k];
      }
      return k;
    };

  // Two samples are unioned into the same cluster ONLY if BOTH their
  // position AND orientation agree within tolerance — see this method's
  // header doc comment for why position-only clustering left a camera
  // roll error uncorrected.
  const double position_bucket_size_m = position_bucket_size_cm / 100.0;
  for (size_t a = 0; a < indices.size(); ++a) {
    for (size_t b = a + 1; b < indices.size(); ++b) {
      const geometry_msgs::msg::Vector3 & pa = collected_positions_[indices[a]];
      const geometry_msgs::msg::Vector3 & pb = collected_positions_[indices[b]];
      const double dx = pa.x - pb.x;
      const double dy = pa.y - pb.y;
      const double dz = pa.z - pb.z;
      const bool position_agrees =
        std::sqrt(dx * dx + dy * dy + dz * dz) <= position_bucket_size_m;

      const double orientation_deviation_deg = angularDeviationDeg(
        collected_orientations_[indices[a]], collected_orientations_[indices[b]]);
      const bool orientation_agrees = orientation_deviation_deg <= orientation_bucket_size_deg;

      if (position_agrees && orientation_agrees) {
        const size_t root_a = find(a);
        const size_t root_b = find(b);
        if (root_a != root_b) {
          parent[root_a] = root_b;
        }
      }
    }
  }

  // Group local indices by cluster root, find the largest cluster.
  std::map<size_t, std::vector<size_t>> clusters;
  for (size_t k = 0; k < indices.size(); ++k) {
    clusters[find(k)].push_back(indices[k]);
  }

  const auto largest_cluster_it = std::max_element(
    clusters.begin(), clusters.end(),
    [](const auto & a, const auto & b) {return a.second.size() < b.second.size();});

  RCLCPP_INFO(
    get_logger(), "Clustering: largest cluster has %zu of %zu samples (position bucket "
    "%.2fcm, orientation bucket %.2fdeg)",
    largest_cluster_it->second.size(), indices.size(), position_bucket_size_cm,
    orientation_bucket_size_deg);

  return plainAverage(largest_cluster_it->second);
}

void CalibrationBroadcasterNode::finishCalibration(
  const std::shared_ptr<GoalHandleCalibrate> & goal_handle)
{
  // Runs BEFORE rejectOutliers()/averaging, so both operate on a yaw/roll
  // signal with fake (corner-detection-noise-induced) variation already
  // removed — see clampYawRoll()'s and config_.yaw_roll_clamp_enabled's
  // own doc comments. Overwrites collected_orientations_ in place (a
  // no-op when the clamp is disabled — clampYawRoll returns its input
  // unchanged) since rejectOutliers()/every downstream call reads that
  // member vector directly, not a passed-in argument.
  collected_orientations_ = clampYawRoll(collected_orientations_);

  const std::vector<size_t> kept_indices = rejectOutliers();
  const bool rejection_changed_anything = kept_indices.size() != collected_positions_.size();

  // use_clustering_average is READ LIVE here, not cached in config_ —
  // deliberately, so the web UI's DevSpace drawer switch takes effect on
  // the very next ~/calibrate run without a node restart, unlike every
  // other field in CalibrationBroadcasterConfig (see that struct's own
  // comment on this field).
  const bool use_clustering = get_parameter("use_clustering_average").as_bool();

  geometry_msgs::msg::Vector3 average_position;
  tf2::Quaternion averaged_orientation;
  // The set of indices actually contributing to the final average — either
  // every kept_index (Mean method) or just the winning cluster's members
  // (Clustering method, 2026-07-29 — see computeClusteredPose()'s doc
  // comment for why orientation is now part of clustering too, not just
  // position). Spread/is_high_confidence below is computed against
  // whichever set actually produced the broadcast result, not always the
  // full kept_indices — a low spread among only the winning cluster's
  // members is the whole point of clustering, and reporting spread against
  // the UNCLUSTERED full set would defeat that.
  std::vector<size_t> contributing_indices;

  if (use_clustering) {
    const ClusteredPose clustered = computeClusteredPose(
      kept_indices, config_.clustering_bucket_size_cm, config_.clustering_bucket_angle_deg);
    average_position = clustered.position;
    averaged_orientation = clustered.orientation;
    contributing_indices = clustered.member_indices;
  } else {
    for (const size_t i : kept_indices) {
      average_position.x += collected_positions_[i].x;
      average_position.y += collected_positions_[i].y;
      average_position.z += collected_positions_[i].z;
    }
    const double count = static_cast<double>(kept_indices.size());
    average_position.x /= count;
    average_position.y /= count;
    average_position.z /= count;

    std::vector<tf2::Quaternion> kept_orientations;
    kept_orientations.reserve(kept_indices.size());
    for (const size_t i : kept_indices) {
      kept_orientations.push_back(collected_orientations_[i]);
    }
    averaged_orientation = averageQuaternions(kept_orientations, averaging_method_).averaged;
    contributing_indices = kept_indices;
  }

  // Post-rejection (and post-clustering, if active) spread — computed
  // against contributing_indices (whichever set actually produced
  // average_position/averaged_orientation) and the FINAL averaged values,
  // needed for is_high_confidence below. This answers "how much does the
  // result we're ACTUALLY broadcasting disagree with the samples that went
  // into it," not a general unfiltered spread statistic.
  double max_position_spread_cm = 0.0;
  double max_orientation_spread_deg = 0.0;
  for (const size_t i : contributing_indices) {
    const geometry_msgs::msg::Vector3 & position = collected_positions_[i];
    const double dx = position.x - average_position.x;
    const double dy = position.y - average_position.y;
    const double dz = position.z - average_position.z;
    max_position_spread_cm =
      std::max(max_position_spread_cm, std::sqrt(dx * dx + dy * dy + dz * dz) * 100.0);

    max_orientation_spread_deg = std::max(
      max_orientation_spread_deg,
      angularDeviationDeg(collected_orientations_[i], averaged_orientation));
  }

  // orientation_result kept for its mean_spread_deg (still reported/logged
  // below) — max_spread_deg is superseded by max_orientation_spread_deg
  // above, which (unlike this) is computed against contributing_indices
  // and the FINAL averaged_orientation, not always the full kept set.
  std::vector<tf2::Quaternion> kept_orientations_for_mean;
  kept_orientations_for_mean.reserve(kept_indices.size());
  for (const size_t i : kept_indices) {
    kept_orientations_for_mean.push_back(collected_orientations_[i]);
  }
  const OrientationAveragingResult orientation_result =
    averageQuaternions(kept_orientations_for_mean, averaging_method_);

  const bool is_high_confidence =
    max_position_spread_cm <= config_.position_spread_tolerance_cm &&
    max_orientation_spread_deg <= config_.orientation_spread_tolerance_deg;

  geometry_msgs::msg::TransformStamped broadcast_tf;
  broadcast_tf.header.stamp = get_clock()->now();
  broadcast_tf.header.frame_id = config_.known_chain_frame;
  // Suffixed, not the detector's raw frame_id: broadcasting under the
  // exact same name as an existing URDF-declared frame (e.g. sim's
  // wrist_rgbd_camera_depth_optical_frame) would conflict with it in the
  // TF tree — two disagreeing publishers for one frame. See
  // CalibrationBroadcasterConfig::broadcast_frame_suffix.
  broadcast_tf.child_frame_id = last_sample_.header.frame_id + config_.broadcast_frame_suffix;
  broadcast_tf.transform.translation = average_position;
  // averaged_orientation (NOT orientation_result.averaged) — this is the
  // actual bug this whole change fixes: orientation_result is always the
  // Mean-method average over every kept sample, regardless of
  // use_clustering_average; broadcasting IT here would silently discard
  // clustering's whole point for orientation specifically (the position
  // would correctly reflect the winning cluster, but the rotation would
  // still be dragged by orientation outliers clustering was supposed to
  // exclude — exactly the camera roll error this change was written to
  // fix). averaged_orientation is the one that's actually
  // clustering-aware (equal to orientation_result.averaged when
  // use_clustering_average is false, since contributing_indices ==
  // kept_indices in that branch — see the if/else above).
  broadcast_tf.transform.rotation = tf2::toMsg(averaged_orientation);

  static_broadcaster_.sendTransform(broadcast_tf);

  if (rejection_changed_anything) {
    // Only meaningful to compute/log the pre-rejection spread when
    // rejection actually discarded something — otherwise it's identical to
    // orientation_result and would just be a confusing duplicate log line.
    const OrientationAveragingResult unfiltered_orientation_result =
      averageQuaternions(collected_orientations_, averaging_method_);
    RCLCPP_INFO(
      get_logger(), "Outlier rejection discarded %zu of %zu samples — orientation spread "
      "improved from max %.3f/mean %.3f deg to max %.3f/mean %.3f deg",
      collected_positions_.size() - kept_indices.size(), collected_positions_.size(),
      unfiltered_orientation_result.max_spread_deg, unfiltered_orientation_result.mean_spread_deg,
      orientation_result.max_spread_deg, orientation_result.mean_spread_deg);
  }

  RCLCPP_INFO(
    get_logger(), "Calibration complete: broadcasting static TF '%s' -> '%s' "
    "(position+orientation via %s, %zu contributing samples of %zu kept; spread vs. "
    "broadcast result: max position %.2fcm, max orientation %.3fdeg; %s)",
    config_.known_chain_frame.c_str(), broadcast_tf.child_frame_id.c_str(),
    use_clustering ? "clustering (position+orientation)" : "mean", contributing_indices.size(),
    kept_indices.size(), max_position_spread_cm, max_orientation_spread_deg,
    is_high_confidence ? "HIGH CONFIDENCE" : "LOW CONFIDENCE (spread exceeds tolerance)");

  auto result = std::make_shared<Calibrate::Result>();
  result->success = true;
  result->message = "Broadcasting static TF '" + config_.known_chain_frame + "' -> '" +
    broadcast_tf.child_frame_id + "'";
  // max_orientation_spread_deg (NOT orientation_result.max_spread_deg) —
  // same reasoning as broadcast_tf.transform.rotation above: this must
  // reflect spread against the ACTUAL broadcast result, which
  // orientation_result does not when clustering is active.
  result->max_spread_deg = max_orientation_spread_deg;
  result->mean_spread_deg = orientation_result.mean_spread_deg;
  result->is_high_confidence = is_high_confidence;
  goal_handle->succeed(result);

  collected_positions_.clear();
  collected_orientations_.clear();
  stable_agreement_count_ = 0;
}

CalibrationBroadcasterConfig CalibrationBroadcasterNode::loadConfigFromParams() const
{
  CalibrationBroadcasterConfig config;
  config.marker_pose_topic = get_parameter("marker_pose_topic").as_string();
  config.known_chain_frame = get_parameter("known_chain_frame").as_string();
  config.marker_frame = get_parameter("marker_frame").as_string();
  config.broadcast_frame_suffix = get_parameter("broadcast_frame_suffix").as_string();
  config.num_samples = static_cast<int>(get_parameter("num_samples").as_int());
  config.sample_wait_timeout_sec = get_parameter("sample_wait_timeout_sec").as_double();

  const std::string mode_name = get_parameter("planning_mode").as_string();
  if (mode_name == "cartesian") {
    config.planning_mode = visual_calibration_msgs::srv::TracePath::Request::PLANNING_MODE_CARTESIAN;
  } else if (mode_name == "joint_space") {
    config.planning_mode =
      visual_calibration_msgs::srv::TracePath::Request::PLANNING_MODE_JOINT_SPACE;
  } else {
    throw std::invalid_argument(
            "Unknown planning_mode: '" + mode_name + "' (expected 'cartesian' or 'joint_space')");
  }

  config.orientation_sum_normalize_priority =
    static_cast<int>(get_parameter("orientation_sum_normalize_priority").as_int());
  config.orientation_markley_priority =
    static_cast<int>(get_parameter("orientation_markley_priority").as_int());

  config.random_phase_samples =
    static_cast<int>(get_parameter("random_phase_samples").as_int());
  config.random_phase_max_offset_m = get_parameter("random_phase_max_offset_m").as_double();
  config.random_phase_max_consecutive_failures =
    static_cast<int>(get_parameter("random_phase_max_consecutive_failures").as_int());

  config.position_spread_tolerance_cm =
    get_parameter("position_spread_tolerance_cm").as_double();
  config.orientation_spread_tolerance_deg =
    get_parameter("orientation_spread_tolerance_deg").as_double();
  config.stable_agreement_count =
    static_cast<int>(get_parameter("stable_agreement_count").as_int());

  config.orientation_sweep_enabled = get_parameter("orientation_sweep_enabled").as_bool();
  config.orientation_sweep_angle_deg =
    get_parameter("orientation_sweep_angle_deg").as_double();

  config.outlier_rejection_enabled = get_parameter("outlier_rejection_enabled").as_bool();
  config.outlier_position_threshold_cm =
    get_parameter("outlier_position_threshold_cm").as_double();
  config.outlier_orientation_threshold_deg =
    get_parameter("outlier_orientation_threshold_deg").as_double();

  config.samples_per_waypoint =
    static_cast<int>(get_parameter("samples_per_waypoint").as_int());

  // hybrid_per_waypoint_enabled deliberately NOT read here (2026-08-04,
  // changed from restart-only) — see CalibrationBroadcasterConfig's own
  // doc comment on this: it's read LIVE via isHybridPerWaypointEnabled()
  // at the point of use instead, same convention as use_clustering_average.

  // get_parameter_or since detect_call_timeout_sec is absent from sim's
  // yaml entirely (real-only, see this field's own doc comment) and
  // automatically_declare_parameters_from_overrides(true) means
  // get_parameter() on an undeclared key would throw.
  get_parameter_or("detect_call_timeout_sec", config.detect_call_timeout_sec, 30.0);

  // get_parameter_or (not get_parameter) — same reasoning as
  // hybrid_per_waypoint_enabled directly above: absent from sim's yaml,
  // real-only. Default 0 = today's original strict behavior (see this
  // field's own doc comment).
  get_parameter_or("min_samples_to_finish", config.min_samples_to_finish, 0);

  config.clustering_bucket_size_cm =
    get_parameter("clustering_bucket_size_cm").as_double();
  config.clustering_bucket_angle_deg =
    get_parameter("clustering_bucket_angle_deg").as_double();
  // use_clustering_average is intentionally NOT read here — see
  // CalibrationBroadcasterConfig's own comment on why it must stay a live
  // get_parameter() call at the point of use in finishCalibration(),
  // never cached into this struct.

  config.yaw_roll_clamp_enabled = get_parameter("yaw_roll_clamp_enabled").as_bool();

  // get_parameter_or (not get_parameter) since these are absent from
  // sim's yaml entirely (real-only test hook — see this field's own doc
  // comment) and automatically_declare_parameters_from_overrides(true)
  // means get_parameter() on an undeclared key would throw.
  get_parameter_or(
    "yaw_roll_clamp_forced_yaw_deg", config.yaw_roll_clamp_forced_yaw_deg,
    std::numeric_limits<double>::quiet_NaN());
  get_parameter_or(
    "yaw_roll_clamp_forced_roll_deg", config.yaw_roll_clamp_forced_roll_deg,
    std::numeric_limits<double>::quiet_NaN());

  return config;
}

}  // namespace aruco_perception