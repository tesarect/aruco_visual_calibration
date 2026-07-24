#ifndef ORCHESTRATOR__CALIBRATION_ORCHESTRATOR_NODE_HPP_
#define ORCHESTRATOR__CALIBRATION_ORCHESTRATOR_NODE_HPP_

#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <visual_calibration_msgs/action/auto_calibrate.hpp>
#include <visual_calibration_msgs/action/calibrate.hpp>
#include <visual_calibration_msgs/msg/auto_calibrate_status.hpp>
#include <visual_calibration_msgs/msg/detection2_d_array.hpp>
#include <visual_calibration_msgs/srv/get_polygon_waypoints.hpp>
#include <visual_calibration_msgs/srv/get_standoff_pose.hpp>
#include <visual_calibration_msgs/srv/move_to_preset.hpp>
#include <visual_calibration_msgs/srv/set_detector_mode.hpp>
#include <visual_calibration_msgs/srv/trace_path.hpp>

namespace orchestrator
{

/// Tuning for CalibrationOrchestratorNode, loaded from a parameter file.
/// Neither trajectory_planner nor calibration_broadcaster_node are told
/// this orchestration exists — trajectory_planner only ever sees ordinary
/// ~/get_standoff_pose/~/trace_path calls (same calls a human/web operator
/// would issue by hand today), and calibration_broadcaster_node only ever
/// sees an ordinary ~/calibrate goal. All new sequencing logic lives here.
struct OrchestratorConfig
{
  /// How long to wait after reaching cal_ready/standoff before starting
  /// auto-centering (if enabled) or calibration — gives the camera a
  /// moment to settle beyond trajectory_planner's own per-waypoint
  /// waypoint_settle_seconds (that one already fires inside the trace_path
  /// call this node makes; this is an *additional* pause specific to the
  /// "about to start a multi-minute calibration run" moment, kept
  /// separately tunable rather than reusing trajectory_planner's param).
  double post_cal_ready_settle_seconds = 2.0;
  /// Master switch for the auto-centering safety step (see
  /// runAutoCenterProbe) — default false. Overridable at runtime via a
  /// standard ROS set_parameters call (same pattern the web app already
  /// uses for calibration_broadcaster_node's planning_mode), so the web
  /// app can flip this without a code change or restart. NOTE: this field
  /// of config_ is only used for the startup ready-log line — the actual
  /// per-run check in executeAutoCalibrate() reads get_parameter(
  /// "auto_center_enabled") live instead of this cached copy, specifically
  /// so a runtime toggle takes effect on the very next ~/auto_calibrate
  /// run without needing loadConfigFromParams() to re-run (fixed
  /// 2026-07-22 — this field used to be read directly and silently
  /// ignored any post-startup toggle, since nothing ever refreshed it).
  bool auto_center_enabled = false;
  /// Distance (meters) moved per probe step, in the standoff pose's own
  /// local X/Y plane (same convention as trajectory_planner's
  /// polygon_radius_m), when searching for each of the four +X/-X/+Y/-Y
  /// visibility boundaries.
  double auto_center_probe_step_m = 0.05;
  /// Maximum probe distance (meters) from cal_ready along any single axis
  /// direction before giving up on that direction and treating cal_ready's
  /// own position as the boundary — keeps the probe search bounded even if
  /// the marker never disappears (e.g. a very wide camera FOV).
  double auto_center_max_probe_m = 0.15;
  /// How long to wait for a marker_pose message after each probe move
  /// before concluding the marker is not visible there.
  double auto_center_visibility_timeout_sec = 2.0;
  /// Which planning mode to request on every ~/trace_path call this node
  /// makes (cal_ready move + auto-center probe moves) — see
  /// TracePath::Request::PLANNING_MODE_*.
  uint8_t planning_mode =
    visual_calibration_msgs::srv::TracePath::Request::PLANNING_MODE_JOINT_SPACE;

  // --- Image-based centering (2026-07-23) — replaces runAutoCenterProbe.
  // auto_center_probe_step_m/auto_center_max_probe_m/
  // auto_center_visibility_timeout_sec above are now unused by the active
  // centering path (kept, along with runAutoCenterProbe/
  // probeDirectionVisible themselves, unreferenced rather than deleted,
  // in case this new approach needs a fallback later).
  //
  // Algorithm (2026-07-23, replaces two earlier attempts — a per-axis
  // repeated-halving search, then a per-axis calibrate-then-jump search —
  // both of which assumed arm-local X only moves the marker along
  // image-X and arm-local Y only moves it along image-Y, i.e. a diagonal
  // mapping. Live testing showed arm-local X producing only 1-6px/m of
  // image-X motion no matter how large the probe step, which is the
  // signature of a camera mounted at an angle relative to the arm's
  // motion plane: X's motion was likely projecting mostly onto image-Y
  // instead, which the old per-axis design had no way to see or use,
  // since it only ever measured the one image axis it was currently
  // searching): uncalibrated Image-Based Visual Servoing (IBVS) via an
  // estimated 2x2 image Jacobian J (interaction matrix), the standard
  // technique from the "uncalibrated"/"calibration-free" visual servoing
  // literature (Jägersand 1996/97, Hosoda & Asada 1994) for regulating a
  // visual feature to a target via 2D pixel feedback alone, with no
  // camera intrinsics/depth/known camera-robot transform. Bootstrapped
  // from 2 linearly-independent probes (pure local +X, then pure local
  // +Y), each measuring the FULL 2D pixel response (not just one axis),
  // giving J's two columns directly (Δs/step per probe — no matrix
  // inversion needed to BUILD J, since the probes are already axis
  // aligned in arm-space). Centering itself then solves
  // Δq = J⁻¹·(target - current) for a direct one-shot jump, refining J
  // via a Broyden update (the standard secant-method generalization used
  // throughout this literature) from each subsequent move's actual
  // measured effect if not yet converged. See centerOnMarkerUsingImage.
  /// Size (meters) of each of the 2 fixed bootstrap probes used to
  /// estimate the image Jacobian — must be big enough to move the marker
  /// a clearly measurable number of pixels on-screen, or the estimated
  /// Jacobian is indistinguishable from detector noise (2026-07-23 —
  /// found live: 0.05m only produced ~4-12px/m at this cell's camera
  /// geometry). Bumped 0.05 -> 0.10 for a stronger default signal;
  /// centering_min_jacobian_column_px_per_m's conditioning check handles
  /// the remaining risk of this still being too small for some other
  /// camera geometry.
  double centering_step_m = 0.10;
  /// An axis is considered centered once the marker's pixel centroid is
  /// within this many pixels of the image's own center along that axis
  /// (checked on BOTH image axes simultaneously, not sequentially).
  double centering_pixel_tolerance = 15.0;
  /// Minimum |pixel response| (Euclidean norm of a probe's measured Δs)
  /// each bootstrap probe must show before its Jacobian column is
  /// trusted — below this, that probe's signal is noise-dominated and
  /// the whole bootstrap is treated as a failure (unlike the old per-axis
  /// design's escalate-and-retry, a degenerate bootstrap here is surfaced
  /// directly rather than retried with a bigger step, since
  /// centering_step_m is already chosen to be as large as is safe/sane —
  /// see that field's own doc comment). Deliberately conservative — real
  /// low-light/lower-resolution detection may be noisier than sim.
  double centering_min_jacobian_column_px_per_m = 20.0;
  /// Safety bound: max total moves (2 bootstrap probes + correction
  /// jumps) before giving up on the whole centering attempt as a
  /// failure. No longer "per axis" — both image axes are corrected
  /// together in each jump.
  int centering_max_iterations = 6;
  /// How long to wait for a fresh Detection2D (aruco_marker) after each
  /// centering step before concluding the marker isn't visible there —
  /// same role/timescale as auto_center_visibility_timeout_sec, reused
  /// under its own name since this is a distinct feature now.
  double centering_visibility_timeout_sec = 2.0;
  /// Safety cap (meters) on any single Jacobian-computed jump — protects
  /// against a near-singular J (e.g. from real-world nonlinearity after
  /// a Broyden update) producing a wild estimated distance. Deliberately
  /// a SEPARATE parameter from auto_center_max_probe_m (that one belongs
  /// to the superseded runAutoCenterProbe path, kept only as an unused
  /// fallback reference — see that field's own doc comment) rather than
  /// reusing it, since the two features' safe-distance tuning is not
  /// guaranteed to match.
  double centering_max_jump_m = 0.15;
  /// CameraInfo topic this node reads ONCE (at first receipt) purely to
  /// learn the image's own pixel width/height (for computing the image's
  /// center point) — no camera intrinsics/depth/TF dependency here at all,
  /// unlike the pinhole-projection design this empirical approach replaced
  /// (see class doc comment). Must match aruco_detector_{sim,real}.yaml's
  /// camera_info_topic for the environment this node is running in.
  std::string camera_info_topic;
};

/// Orchestrates the full auto-calibrate sequence behind one action,
/// ~/auto_calibrate:
///   1. Move to cal_ready/standoff (trajectory_planner's
///      ~/get_standoff_pose, then ~/trace_path with the result — the exact
///      two calls the web app's "Cal Ready" button already makes today).
///   2. Wait post_cal_ready_settle_seconds.
///   3. If auto_center_enabled: probe +X/-X/+Y/-Y from cal_ready (see
///      runAutoCenterProbe) to find how far the marker stays visible in
///      each direction, then move to the midpoint of the surviving
///      boundaries — trajectory_planner's own ~/get_standoff_pose/polygon
///      math re-centers on wherever the arm is now, since it always
///      recomputes the standoff fresh from the live camera TF (see
///      TrajectoryPlanner::polygonWaypointsAroundStandoff) — no new API
///      needed on trajectory_planner's side to "tell" it about the
///      corrected center.
///   4. Call calibration_broadcaster_node's existing ~/calibrate action as
///      a client, relaying its feedback/result through this action.
///
/// Neither trajectory_planner nor calibration_broadcaster_node change to
/// support this — this node only ever calls their already-existing public
/// interfaces, the same ones a human/web operator could call by hand.
///
/// Also exposes ~/start_auto_calibrate (std_srvs/Trigger) +
/// ~/auto_calibrate_status (visual_calibration_msgs/AutoCalibrateStatus,
/// plain topic) as a rosbridge-reachable facade in front of the
/// ~/auto_calibrate action above — see handleStartAutoCalibrate's doc
/// comment for why (rosbridge_suite 1.3.1, this project's version, has NO
/// ROS2 action support at all — confirmed 2026-07-20). The action itself
/// is unchanged and still directly usable by any ROS2-native client (e.g.
/// `ros2 action send_goal`); the facade is additive, not a replacement.
///
/// Also exposes ~/set_detector_mode (visual_calibration_msgs/
/// SetDetectorMode) — the classical/hybrid detector switch. Neither
/// aruco_perception's ArucoDetectorNode nor aruco_perception_yolo_bridge's
/// YoloMarkerBridgeNode know this orchestration exists, same philosophy as
/// the rest of this class: this node just flips one node's "active"
/// parameter true and the other's false via two rclcpp::AsyncParametersClient
/// instances (one per detector node) and the standard ROS set_parameters
/// service — no lifecycle nodes, no process start/stop, both detector
/// nodes stay running/subscribed the whole time. See handleSetDetectorMode.
class CalibrationOrchestratorNode : public rclcpp::Node
{
public:
  using AutoCalibrate = visual_calibration_msgs::action::AutoCalibrate;
  using GoalHandleAutoCalibrate = rclcpp_action::ServerGoalHandle<AutoCalibrate>;
  using Calibrate = visual_calibration_msgs::action::Calibrate;

  CalibrationOrchestratorNode();

private:
  OrchestratorConfig loadConfigFromParams() const;

  /// Caches the latest marker_pose message's receipt time — used only by
  /// runAutoCenterProbe to check "is the marker visible right now", not
  /// for calibration sampling (calibration_broadcaster_node subscribes to
  /// the same topic independently for that).
  void markerPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr & msg);

  /// Clears pending_manual_adjustment_ whenever trajectory_planner reports
  /// (via /trajectory_planner/current_pose_name) that the arm moved to ANY
  /// named preset pose (2026-07-24) — e.g. the user pressing "Home" or
  /// "Standby" instead of fine-tuning further is treated as abandoning
  /// that in-progress adjustment. Does NOT itself trigger on "cal_ready"
  /// specifically vs. any other name — any transition on this topic means
  /// the arm is now at a well-known preset, not a manually fine-tuned
  /// pose, so the flag is cleared unconditionally on every message.
  void currentPoseNameCallback(const std_msgs::msg::String::ConstSharedPtr & msg);

  /// Handles ~/start_auto_calibrate (std_srvs/Trigger) — a rosbridge-
  /// reachable facade in front of ~/auto_calibrate, for clients that can't
  /// speak rosbridge's native ROS2 action protocol (see
  /// AutoCalibrateStatus.msg's header comment: rosbridge_suite 1.3.1 has
  /// NO action support at all, confirmed 2026-07-20 via its own
  /// "Unknown operation: send_action_goal" rejection). Sends a normal
  /// AutoCalibrate goal to THIS node's own action server via
  /// self_action_client_ (a supported pattern — a node can be a client of
  /// its own action server), relays feedback/result onto
  /// ~/auto_calibrate_status (see publishStatus*), and returns
  /// success=true immediately once the goal is ACCEPTED — this is
  /// fire-and-forget from the service caller's perspective, not
  /// fire-and-wait-for-completion; progress/result only ever arrive via
  /// the status topic. Returns success=false (Trigger's response) only if
  /// goal submission itself failed (e.g. action server unreachable) —
  /// never if the calibration sequence itself later fails, since that's
  /// reported via the status topic instead.
  void handleStartAutoCalibrate(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /// Handles ~/cancel_auto_calibrate (std_srvs/Trigger) — the cancel-side
  /// half of the same rosbridge facade as handleStartAutoCalibrate (see
  /// its doc comment). Cancels the goal tracked in
  /// self_action_goal_handle_ (set once the goal-response callback in
  /// handleStartAutoCalibrate confirms acceptance) via
  /// self_action_client_->async_cancel_goal(goal_handle) — see
  /// rclcpp_action::Client. Returns success=false if there is no in-flight
  /// goal to cancel (nothing started yet, or the previous run already
  /// finished) — this is a legitimate "nothing to do" outcome, not
  /// treated as a server error.
  void handleCancelAutoCalibrate(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /// Handles ~/set_detector_mode. request.mode must be "classical" or
  /// "hybrid" (anything else is rejected with success=false, response.mode
  /// left unset — a validation failure, not a node-unreachable failure).
  /// Sets active=true on the target node's AsyncParametersClient FIRST,
  /// THEN active=false on the other — briefly both-active rather than
  /// briefly neither-active, since a duplicate marker_pose publish for one
  /// frame is harmless (calibration_broadcaster_node just sees an extra
  /// sample) while a gap where NEITHER publishes is a real, if brief, loss
  /// of the pose stream. Returns success=false (with a descriptive
  /// response.message) if either set_parameters call fails or times out —
  /// does NOT partially apply the switch: if the second call fails after
  /// the first succeeded, logs the resulting inconsistent state rather
  /// than silently leaving it unclear which detector a caller should
  /// believe is authoritative.
  void handleSetDetectorMode(
    const std::shared_ptr<visual_calibration_msgs::srv::SetDetectorMode::Request> request,
    std::shared_ptr<visual_calibration_msgs::srv::SetDetectorMode::Response> response);

  /// Publishes one AutoCalibrateStatus with phase=PHASE_RUNNING and the
  /// given feedback fields (result fields left zero-valued) — called from
  /// self_action_client_'s feedback callback, mirroring exactly what
  /// publish_stage's lambda inside executeAutoCalibrate already sends as
  /// native action feedback (see that function) — this is a parallel
  /// broadcast, not a replacement.
  void publishStatusFeedback(const AutoCalibrate::Feedback & feedback);

  /// Publishes one AutoCalibrateStatus with phase=PHASE_SUCCEEDED or
  /// PHASE_FAILED (per result.success) and the given result fields
  /// (feedback fields left zero-valued) — called from
  /// self_action_client_'s result callback.
  void publishStatusResult(const AutoCalibrate::Result & result);

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const AutoCalibrate::Goal> goal);

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleAutoCalibrate> goal_handle);

  /// Spawns a detached thread running executeAutoCalibrate(goal_handle) —
  /// rclcpp_action requires handleAccepted to return quickly, not block.
  void handleAccepted(const std::shared_ptr<GoalHandleAutoCalibrate> goal_handle);

  /// The actual sequence, run on its own thread — see class doc comment
  /// for the 4 stages. Aborts (goal_handle->abort, with failed_stage set)
  /// on the first stage that fails.
  void executeAutoCalibrate(const std::shared_ptr<GoalHandleAutoCalibrate> goal_handle);

  /// Stage 1: tries trajectory_planner's ~/move_to_preset("cal_ready")
  /// FIRST — if a joint-value preset named "cal_ready" is configured (see
  /// preset_poses_sim.yaml/_real.yaml, TrajectoryPlanner::planAndExecuteToPreset),
  /// this pins the exact IK branch/joint configuration instead of leaving
  /// it to whatever solution a fresh ~/get_standoff_pose + ~/trace_path
  /// call happens to land on. Confirmed 2026-07-20: two different
  /// joint-space paths to the SAME cal_ready Cartesian pose produced joint
  /// configurations differing by 90-250° on several joints, only one of
  /// which left enough margin for the downstream Cartesian polygon-corner
  /// moves (runAutoCenterProbe, calibration_broadcaster_node's sampling)
  /// to succeed rather than fail partway (computeCartesianPath() stopping
  /// at ~80-83%).
  /// Falls back to the ORIGINAL behavior — ~/get_standoff_pose then
  /// ~/trace_path with the result (pose_name "cal_ready", is_sequenced_goal
  /// false — same call shape the web app's "Cal Ready" button already
  /// makes) — if move_to_preset reports no "cal_ready" preset is configured
  /// (e.g. real, which has no joint-value preset captured yet as of
  /// 2026-07-20, only its existing Cartesian one). Either way, returns the
  /// resulting Cartesian pose on success (queried via ~/get_standoff_pose
  /// after a successful joint-preset move — that call is read-only/
  /// no-motion, see TrajectoryPlanner::getStandoffPose — since
  /// runAutoCenterProbe needs the actual Cartesian pose regardless of which
  /// path got the arm there), or std::nullopt on failure (logs the reason).
  std::optional<geometry_msgs::msg::Pose> moveToCalReady();

  /// Read-only "what is the arm's current Cartesian pose right now" query
  /// (2026-07-24) — used by executeAutoCalibrate when goal->skip_cal_ready
  /// is true, to get a pose to hand to centerOnMarkerUsingImage/
  /// runCalibrate without moving the arm anywhere first (e.g. after a user
  /// manually fine-tuned position/orientation via the web app's control
  /// drawer, then hit "Calibrate from current pose" instead of the normal
  /// "Calibrate", which always returns to cal_ready first). Calls
  /// trajectory_planner's ~/get_polygon_waypoints and uses ONLY its
  /// response's center_pose field — the waypoints themselves are
  /// discarded. Reuses this service specifically because its center_pose
  /// is already a proven-safe current-pose read (TF lookup of
  /// end_effector_frame, not MoveGroupInterface::getCurrentState() —  see
  /// TrajectoryPlanner::polygonWaypointsAroundStandoff's 2026-07-22 fix
  /// for a callback-group deadlock that approach used to hit), rather than
  /// adding a new dedicated service/TF buffer to this node for the same
  /// read. Returns std::nullopt on failure (logs the reason).
  std::optional<geometry_msgs::msg::Pose> getCurrentArmPose();

  /// Returns true if a marker_pose message was received within
  /// config_.auto_center_visibility_timeout_sec of `after`.
  bool isMarkerVisibleAfter(const rclcpp::Time & after);

  /// SUPERSEDED (2026-07-23) by centerOnMarkerUsingImage — kept
  /// unreferenced (not deleted) rather than removed, in case the new
  /// image-based approach needs a fallback later. Stage 3 used to be:
  /// axis-by-axis recentering from center_pose (cal_ready's pose). First
  /// probes +X/-X in center_pose's own local X/Y plane in
  /// config_.auto_center_probe_step_m increments (each probe: move via
  /// ~/trace_path, then isMarkerVisibleAfter) until the marker disappears
  /// or config_.auto_center_max_probe_m is reached — that distance becomes
  /// the boundary for that direction (0.0 if the marker wasn't even
  /// visible one step out). Moves to the X-midpoint pose BEFORE probing Y
  /// at all, so the Y probe's own boundaries reflect visibility at the
  /// already-X-corrected position, not the original (possibly off-center)
  /// X. Then repeats the same probe+move pattern for +Y/-Y from that
  /// X-centered pose, landing on the final X-and-Y-centered pose.
  /// Orientation is kept identical to center_pose throughout — only
  /// position changes. On success, also stores the result in
  /// session_centered_cal_ready_pose_ (see its doc comment) so a later
  /// moveToCalReady() call in the same session reuses it instead of
  /// re-deriving cal_ready from scratch. Returns the corrected pose moved
  /// to on success, std::nullopt if any of the required moves failed
  /// outright (NOT if a probe simply finds a 0.0 boundary — a marker not
  /// visible even one step out just means that direction's boundary is
  /// the starting pose itself, which is a valid, if degenerate, result).
  std::optional<geometry_msgs::msg::Pose> runAutoCenterProbe(
    const geometry_msgs::msg::Pose & center_pose);

  /// One probe move + visibility check in the given local-frame direction
  /// (+1/-1 on x_axis xor y_axis, not both) at `distance_m` from
  /// center_pose. Returns true if the marker was visible there. Does not
  /// move back to center_pose — callers issue the next probe or the final
  /// centering move themselves. Logs (2026-07-22) WHICH of the two
  /// possible `false` causes actually occurred — a failed
  /// tracePathBlocking() (plan/execute error, e.g. a joint limit) vs. a
  /// successful move followed by isMarkerVisibleAfter() finding no
  /// marker — since both look identical to runAutoCenterProbe's caller
  /// (same `false` return) but have different implications (a
  /// reachability wall vs. a genuine camera-FOV edge); without this, an
  /// asymmetric probe result across the 4 directions can't be diagnosed
  /// from the log alone.
  bool probeDirectionVisible(
    const geometry_msgs::msg::Pose & center_pose,
    double x_axis, double y_axis, double distance_m);

  /// Stage 3 (2026-07-23, replaces runAutoCenterProbe, and replaces an
  /// interim per-axis calibrate-then-jump design that assumed a diagonal
  /// arm-to-image mapping — see OrchestratorConfig's "Image-based
  /// centering" doc comment for the full algorithm background and why
  /// that assumption broke live testing): uncalibrated IBVS via an
  /// estimated 2x2 image Jacobian. NO camera intrinsics, NO depth, NO TF
  /// lookup to the camera frame at all (the pinhole-projection design
  /// this replaced would have needed a TF lookup to the camera's optical
  /// frame to rotate a computed offset into the arm's local plane, but
  /// that TF doesn't exist on real — see class doc comment).
  ///
  /// 1. Measure the starting pixel offset s0 = (u,v) - image_center via
  ///    latestMarkerPixelAfter()/image_width_/image_height_.
  /// 2. Bootstrap: probe +config_.centering_step_m in local X, then
  ///    (from that new pose) +config_.centering_step_m in local Y, via
  ///    stepAndMeasurePixelOffset() — each measures the FULL 2D pixel
  ///    response, not just one axis. The two measured (Δu,Δv)/step pairs
  ///    are directly the two columns of the image Jacobian J (no matrix
  ///    inversion needed to build it, since the probes are axis-aligned
  ///    in arm-space). If either probe fails (plan/execute error or
  ///    marker invisible) or its pixel response is too weak
  ///    (config_.centering_min_jacobian_column_px_per_m), the bootstrap
  ///    fails outright — see jacobianConditionOk().
  /// 3. Solve Δq = J⁻¹·(target - current) (closed-form 2x2 inverse, see
  ///    invert2x2()) for a single corrective jump, clamped to
  ///    config_.centering_max_jump_m, and execute it.
  /// 4. Remeasure. If within config_.centering_pixel_tolerance on BOTH
  ///    image axes simultaneously, done. Otherwise refine J with a
  ///    Broyden update (secant-method generalization to 2D — standard in
  ///    the uncalibrated visual servoing literature) from this move's
  ///    actual (Δq, Δs), and repeat from step 3 — bounded by
  ///    config_.centering_max_iterations total (bootstrap + corrections).
  ///
  /// On success, stores the result in session_centered_cal_ready_pose_
  /// same as the old method did. Sets/clears show_centering_crosshair on
  /// aruco_detector_node (via AsyncParametersClient, same mechanism as
  /// handleSetDetectorMode) for the duration of the search.
  ///
  /// On failure, out_user_message is set to a short, USER-FACING
  /// suggestion (2026-07-23 — e.g. "couldn't detect the marker
  /// reliably... try improving lighting, moving the camera/marker
  /// closer, or switching to hybrid detection mode") suitable for direct
  /// display in the web app, distinct from the detailed RCLCPP_ERROR
  /// logs above (which stay developer-facing/technical). Left unset
  /// (untouched) on success. Categorizes every internal nullopt-return
  /// site into one of three user-relevant buckets: marker never visible
  /// to begin with, bootstrap couldn't get a reliable signal, or a
  /// corrective move failed/lost the marker mid-search — all three are
  /// plausibly fixed by the same handful of user actions, so one shared
  /// message covers them; only the max-iterations-without-converging
  /// case gets a distinct message (that one isn't a detection problem).
  std::optional<geometry_msgs::msg::Pose> centerOnMarkerUsingImage(
    const geometry_msgs::msg::Pose & center_pose, std::string & out_user_message);

  /// One step-and-remeasure move: offsets current_pose by
  /// (dx_m, dy_m) in its own local X/Y plane (see offsetInLocalPlane),
  /// waits for a fresh Detection2D, and returns the FULL 2D pixel offset
  /// from image center (Δu, Δv) — both axes, regardless of which arm
  /// axis moved — if the marker was visible afterward, or std::nullopt if
  /// the move failed outright OR the marker wasn't visible. Deliberately
  /// returns both axes (2026-07-23 — the old stepAndMeasureAxisOffset
  /// returned only the searched axis, discarding the other axis's
  /// incidental movement, which is exactly the data centerOnMarkerUsingImage's
  /// Jacobian estimate needs to see cross-axis coupling).
  std::optional<std::pair<double, double>> stepAndMeasurePixelOffset(
    const geometry_msgs::msg::Pose & current_pose, double dx_m, double dy_m);

  /// Returns the most recent aruco_marker Detection2D's (cx, cy) if one
  /// was received after `after`, else std::nullopt — polls rather than
  /// blocking (same reasoning as isMarkerVisibleAfter: a step to a
  /// position where the marker is genuinely invisible must time out
  /// gracefully), bounded by config_.centering_visibility_timeout_sec.
  std::optional<std::pair<double, double>> latestMarkerPixelAfter(const rclcpp::Time & after);

  /// Callback for detections_2d_sub_ — caches the most recent aruco_marker
  /// entry's (cx, cy) and receipt time (mirrors markerPoseCallback's
  /// pattern). Ignores frames with no aruco_marker entry (empty
  /// detections[] or only cup_holder/hole present) — leaves the cached
  /// value in place rather than clearing it, so a single momentary gap
  /// doesn't look like "never seen."
  void detections2dCallback(
    const visual_calibration_msgs::msg::Detection2DArray::ConstSharedPtr & msg);

  /// Callback for camera_info_sub_ — captures image_width_/image_height_
  /// ONCE (first receipt only, same one-shot pattern as
  /// ArucoDetectorNode::cameraInfoCallback) purely to compute the image's
  /// own pixel center; nothing else from CameraInfo is used here.
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);

  /// Sends a single-waypoint ~/trace_path request and blocks for the
  /// response — shared by moveToCalReady/runAutoCenterProbe. pose_name/
  /// is_sequenced_goal forwarded as given (both default to the "no
  /// bookkeeping" values used by every caller in this file). Returns
  /// nullopt if the service isn't available or the call fails.
  bool tracePathBlocking(
    const geometry_msgs::msg::Pose & target,
    const std::string & pose_name = "");

  /// Stage 4: sends an empty goal to calibration_broadcaster_node's
  /// ~/calibrate action, relaying its feedback as AutoCalibrate feedback
  /// (stage "Calibrating (sample N/M)") and blocking until it completes.
  /// Returns the Calibrate::Result on success or failure alike (check
  /// ->success) — nullopt only if the action server itself wasn't
  /// reachable.
  std::shared_ptr<Calibrate::Result> runCalibrate(
    const std::shared_ptr<GoalHandleAutoCalibrate> & goal_handle);

  OrchestratorConfig config_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr marker_pose_sub_;
  /// See currentPoseNameCallback — clears pending_manual_adjustment_ on
  /// any preset-pose transition trajectory_planner reports.
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr current_pose_name_sub_;
  /// See detections2dCallback — feeds centerOnMarkerUsingImage's pixel
  /// measurements. Works with EITHER detector as of 2026-07-23 —
  /// yolo_marker_bridge_node.py now also publishes an "aruco_marker"
  /// Detection2D entry (previously only aruco_detector_node did, which
  /// silently made image-based centering fail/time out whenever hybrid
  /// mode was active — closed by adding the matching publish to
  /// yolo_marker_bridge_node.py's publish_detections_2d).
  rclcpp::Subscription<visual_calibration_msgs::msg::Detection2DArray>::SharedPtr
    detections_2d_sub_;
  /// See cameraInfoCallback — read once, purely for image_width_/
  /// image_height_.
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp_action::Server<AutoCalibrate>::SharedPtr auto_calibrate_action_server_;
  rclcpp_action::Client<Calibrate>::SharedPtr calibrate_action_client_;
  /// This node's own client of its own ~/auto_calibrate action server —
  /// see handleStartAutoCalibrate's doc comment for why (rosbridge-facade
  /// support). A perfectly normal, supported rclcpp_action pattern; not
  /// circular in any problematic sense — goal submission/feedback/result
  /// all go through the same executor as any other client would see.
  rclcpp_action::Client<AutoCalibrate>::SharedPtr self_action_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_auto_calibrate_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_auto_calibrate_service_;
  /// See handleSetDetectorMode. Lazily constructed on first use (inside
  /// getClassicalDetectorParamClient/getHybridDetectorParamClient), NOT in
  /// this node's own constructor — construction requires a
  /// std::shared_ptr<Node>, obtained here via shared_from_this(), which
  /// throws if called before this node object is itself already wrapped
  /// in a shared_ptr by whoever created it (true by the time any service
  /// callback runs, NOT true during this node's own constructor body).
  ///
  /// Deliberately rclcpp::AsyncParametersClient, NOT SyncParametersClient
  /// (2026-07-23 — was SyncParametersClient originally). SyncParametersClient's
  /// blocking calls (set_parameters/set_parameters_atomically) internally
  /// call rclcpp::executors::spin_node_until_future_complete(), which
  /// unconditionally does executor.add_node(this)/remove_node(this) around
  /// the wait — see that function's own "does not work recursively; can't
  /// call spin_node_until_future_complete inside a callback executed by an
  /// executor" comment in rclcpp's executors.hpp. Since this node's own
  /// callbacks (e.g. centerOnMarkerUsingImage, called from the
  /// ~/auto_calibrate goal callback) are exactly that — already running
  /// inside a callback this node's own executor is spinning — that
  /// add_node() always throws "Node has already been added to an
  /// executor." Passing our own executor into SyncParametersClient's
  /// executor-taking constructor (an earlier attempted fix, same day)
  /// does NOT help — spin_node_until_future_complete() still calls
  /// add_node() on whichever executor it's given, unconditionally, so the
  /// crash reproduced identically even after that change. Using
  /// AsyncParametersClient + waitForParametersFuture()'s manual
  /// future-polling instead sidesteps this class of bug completely: no
  /// executor spin is ever invoked from within our own callback — the
  /// response is serviced by another MultiThreadedExecutor worker thread
  /// while we just poll the std::shared_future.
  rclcpp::AsyncParametersClient::SharedPtr classical_detector_param_client_;
  rclcpp::AsyncParametersClient::SharedPtr hybrid_detector_param_client_;
  rclcpp::AsyncParametersClient::SharedPtr getClassicalDetectorParamClient();
  rclcpp::AsyncParametersClient::SharedPtr getHybridDetectorParamClient();

  /// Polls (does NOT spin any executor) a parameters-client future until
  /// it's ready or timeout_sec elapses. Safe to call from within a
  /// callback this node's own executor is already spinning — see the
  /// doc comment above classical_detector_param_client_ for why the
  /// SyncParametersClient equivalent is NOT safe here. Returns nullopt on
  /// timeout.
  template<typename ResultT>
  std::optional<ResultT> waitForParametersFuture(
    std::shared_future<ResultT> future, double timeout_sec)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_sec);
    while (std::chrono::steady_clock::now() < deadline) {
      if (future.wait_for(std::chrono::milliseconds(20)) == std::future_status::ready) {
        return future.get();
      }
    }
    return std::nullopt;
  }
  /// Dedicated callback group for set_detector_mode_service_ (2026-07-24)
  /// — see that service's construction site in the constructor for why:
  /// handleSetDetectorMode blocks waiting for AsyncParametersClient
  /// responses, which under this node's shared default callback group
  /// (used everywhere else — no other explicit groups exist) deadlocked
  /// against itself, since the same group also needs to process those
  /// incoming responses.
  rclcpp::CallbackGroup::SharedPtr set_detector_mode_callback_group_;
  rclcpp::Service<visual_calibration_msgs::srv::SetDetectorMode>::SharedPtr
    set_detector_mode_service_;
  /// The in-flight goal from the most recent ~/start_auto_calibrate call,
  /// set once its goal-response callback confirms server-side acceptance
  /// — used by handleCancelAutoCalibrate. Guarded by
  /// self_action_goal_handle_mutex_ since the goal-response callback
  /// (executor thread) and handleCancelAutoCalibrate (a service callback,
  /// possibly a different executor thread under MultiThreadedExecutor)
  /// both touch it. Left non-null after completion (not reset to nullptr
  /// on result) — a cancel call against an already-finished goal is a
  /// harmless no-op via rclcpp_action's own handling, so there's no need
  /// to track "is this goal still active" separately here.
  rclcpp_action::ClientGoalHandle<AutoCalibrate>::SharedPtr self_action_goal_handle_;
  std::mutex self_action_goal_handle_mutex_;
  /// See AutoCalibrateStatus.msg's header comment — plain reliable QoS
  /// (not transient_local): a caller must ~/start_auto_calibrate first,
  /// THEN subscribe, to see this run's progress; there is no meaningful
  /// "last status" to replay to a late subscriber across separate runs.
  rclcpp::Publisher<visual_calibration_msgs::msg::AutoCalibrateStatus>::SharedPtr
    auto_calibrate_status_pub_;
  rclcpp::Client<visual_calibration_msgs::srv::GetStandoffPose>::SharedPtr
    get_standoff_pose_client_;
  rclcpp::Client<visual_calibration_msgs::srv::TracePath>::SharedPtr trace_path_client_;
  rclcpp::Client<visual_calibration_msgs::srv::MoveToPreset>::SharedPtr move_to_preset_client_;
  /// Reused for its response's center_pose field only (2026-07-24) — a
  /// read-only "what is the arm's current Cartesian pose right now" query
  /// (see TrajectoryPlanner::polygonWaypointsAroundStandoff's TF-based
  /// implementation, fixed 2026-07-22 to avoid a MoveGroupInterface
  /// callback-group deadlock — this reuses that already-proven-safe
  /// mechanism rather than adding a new service/TF buffer to this node).
  /// The returned waypoints themselves are discarded — see
  /// getCurrentArmPose().
  rclcpp::Client<visual_calibration_msgs::srv::GetPolygonWaypoints>::SharedPtr
    get_polygon_waypoints_client_;
  /// The most recent auto-centered pose (set by runAutoCenterProbe on
  /// success), if any — treated as this session's effective cal_ready
  /// until the arm is moved elsewhere or a fresh ~/auto_calibrate run
  /// re-probes. In-memory only (not written back to preset_poses_*.yaml):
  /// lost on node restart, same as any other runtime-only state in this
  /// class. moveToCalReady() returns this instead of re-querying
  /// ~/get_standoff_pose when set, so a second Calibrate press in the same
  /// session reuses the corrected center rather than drifting back to the
  /// original (possibly off-marker-center) cal_ready pose.
  std::optional<geometry_msgs::msg::Pose> session_centered_cal_ready_pose_;

  /// True once a ~/auto_calibrate run's auto-centering stage has failed
  /// (2026-07-24) — the user was shown a message suggesting they fine-tune
  /// the pose via the web app's control drawer (see
  /// centerOnMarkerUsingImage's kDetectionTroubleMessage/etc.) and is
  /// expected to press the SAME Calibrate button again afterward. While
  /// this is true, executeAutoCalibrate's Stage 1 calibrates from the
  /// arm's CURRENT pose (via getCurrentArmPose()) instead of returning to
  /// cal_ready first — a single button that behaves automatically based
  /// on this internal state, rather than a separate button/goal field.
  /// Cleared: (a) once a ~/auto_calibrate run reaches Stage 4 (calibrate),
  /// success or failure — see executeAutoCalibrate; (b) whenever
  /// trajectory_planner reports (via ~/current_pose_name,
  /// currentPoseNameCallback) that the arm moved to ANY named preset pose
  /// — the user choosing a different preset button is treated as
  /// abandoning whatever manual fine-tune was in progress. In-memory
  /// only, lost on node restart, same as session_centered_cal_ready_pose_.
  bool pending_manual_adjustment_ = false;

  /// Guards latest_marker_pose_stamp_, notified by markerPoseCallback and
  /// read by isMarkerVisibleAfter. Deliberately separate from
  /// calibration_broadcaster_node's own equivalent state — this node only
  /// needs "was anything received recently", not the pose value itself.
  std::mutex marker_mutex_;
  rclcpp::Time latest_marker_pose_stamp_;

  /// Guards latest_marker_cx_/latest_marker_cy_/latest_marker_pixel_stamp_
  /// — separate mutex from marker_mutex_ (different data, different
  /// producer callback) even though both ultimately serve
  /// centerOnMarkerUsingImage, to avoid any confusion about which lock
  /// protects which field.
  std::mutex marker_pixel_mutex_;
  double latest_marker_cx_ = 0.0;
  double latest_marker_cy_ = 0.0;
  rclcpp::Time latest_marker_pixel_stamp_;

  /// Image dimensions, captured once from camera_info_sub_'s first
  /// message — see cameraInfoCallback.
  int image_width_ = 0;
  int image_height_ = 0;
  bool camera_info_received_ = false;
};

}  // namespace orchestrator

#endif  // ORCHESTRATOR__CALIBRATION_ORCHESTRATOR_NODE_HPP_