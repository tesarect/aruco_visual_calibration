#ifndef DEPTH_PERCEPTION__DEPTH_PERCEPTION_NODE_HPP_
#define DEPTH_PERCEPTION__DEPTH_PERCEPTION_NODE_HPP_

#include <array>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visual_calibration_msgs/msg/auto_calibrate_status.hpp>
#include <visual_calibration_msgs/msg/detection2_d_array.hpp>
#include <visual_calibration_msgs/msg/stable_position_array.hpp>

namespace depth_perception
{

/*
 * Per-environment (sim/real) topic names + back-projection tuning for
 * DepthPerceptionNode, loaded from a parameter file rather than hardcoded
 * — sim's wrist-mounted RGBD sensor and real's wall-mounted D415 (over
 * Zenoh) publish under different topic names, matching the sim/real config
 * split used everywhere else in this project (see aruco_perception's
 * ImageSubscriberConfig).
 */
struct DepthPerceptionConfig
{
  // Topic carrying the color (RGB) image. Not used for any math yet (YOLO
  // already ran on this image upstream, inside yolo_marker_bridge_node) —
  // subscribed here only so this node's own log output can show that the
  // color feed is alive, same as Step 1.
  std::string rgb_image_topic;

  // Topic carrying the depth image (one distance value per pixel).
  std::string depth_image_topic;

  // Topic carrying the sensor_msgs/CameraInfo (focal length, optical
  // center, distortion) needed to convert a 2D pixel + depth value into
  // an actual 3D point relative to the camera.
  std::string camera_info_topic;

  // Topic carrying visual_calibration_msgs/Detection2DArray — the
  // per-frame cup_holder/hole (and aruco_marker) 2D pixel detections
  // published by yolo_marker_bridge_node. See Detection2D.msg's own doc
  // comment: depth_perception is documented as this topic's intended
  // consumer for exactly the back-projection this node performs.
  std::string detections_2d_topic;

  // Multiplies every raw depth-image value to convert it to meters.
  // 32FC1 depth images (Gazebo's libgazebo_ros_camera plugin, confirmed
  // for this project's sim in intel_r430.urdf.xacro, and REP-118's
  // convention generally) are already in meters, so this defaults to 1.0
  // — kept as a parameter (not hardcoded to 1.0) as a one-line escape
  // hatch, in case the real D415's depth topic ever turns out to publish
  // millimeters (e.g. a raw 16UC1 topic) instead of meters.
  double depth_scale_to_meters = 1.0;

  // Half-width (pixels) of the square patch sampled around each
  // detection's (cx, cy) when reading depth — e.g. 2 means a 5x5 patch.
  // A single pixel's depth reading can be invalid/noisy, especially near
  // a cavity rim (see Detection2D.msg's doc comment), so a small
  // neighborhood is sampled and reduced (median) instead of trusting one
  // pixel. Also serves as the ceiling (depth_patch_max_half_size_px is not
  // a separate field — see backProjectDetection's own doc comment) for the
  // radius-scaled patch size computed per-detection below, so a large
  // detection (e.g. cup_holder) never samples a bigger neighborhood than
  // this proven-safe value.
  int depth_patch_half_size_px = 2;

  // --- radius-scaled depth-sampling patch (2026-08-03) ---
  // See backProjectDetection's own doc comment for the full rationale: a
  // FIXED patch half-size (depth_patch_half_size_px above) can straddle a
  // small hole's rim, biasing its sampled depth toward the rim/wall rather
  // than the true floor. These three params scale the patch actually used
  // down to a fraction of each detection's own bbox-derived radius instead,
  // while a hole/cup_holder large enough for the fixed size to be safe
  // simply gets clamped back up to it (see depth_patch_half_size_px above,
  // reused here as the max clamp — not duplicated as its own field).
  //
  // Defaulted so that, out of the box, EVERY detection's computed patch
  // half-size clamps to depth_patch_half_size_px regardless of its actual
  // radius — i.e. byte-identical behavior to the old fixed-size-only code.
  // Tune radius_scale_factor/min down only after the new diagnostic log
  // lines (see backProjectDetection) confirm this bug is actually present
  // in a given environment — see the fix's plan doc for the full
  // log-first-then-tune sequencing rationale.

  // Fraction of a detection's own bbox-derived radius to use as its patch
  // half-size, before clamping to [depth_patch_min_half_size_px,
  // depth_patch_half_size_px]. Default 1.0 is a deliberate placeholder,
  // NOT a considered "good" scale factor — it's set specifically so the
  // clamp below (max = depth_patch_half_size_px = 2px) is what actually
  // constrains every real detection's patch size at startup, keeping this
  // param a true no-op until tuned. A real fix will very likely want this
  // well below 1.0 (e.g. 0.4-0.5), so a small hole's patch stays safely
  // inside its own radius instead of merely being capped at today's fixed
  // value.
  double depth_patch_radius_scale_factor = 1.0;

  // Floor on the computed patch half-size (pixels) — never shrinks below
  // this even for a very small detected radius, so the sampled patch is
  // never a single, individually-unstable pixel. 1 means at minimum a 3x3
  // patch.
  int depth_patch_min_half_size_px = 1;

  // Topic carrying calibration_orchestrator_node's
  // visual_calibration_msgs/AutoCalibrateStatus — this node watches it
  // purely to know when a ~/auto_calibrate run is actively in progress
  // (phase == PHASE_RUNNING), see pause_while_calibration below.
  std::string auto_calibrate_status_topic;

  // When true (the default), this node stops processing detections_2d/
  // depth entirely for the duration of an active ~/auto_calibrate run
  // (phase == PHASE_RUNNING on auto_calibrate_status_topic), freeing up
  // CPU for marker detection — which calibration's sampling loop actually
  // depends on being fast/reliable — instead of competing with it for the
  // same YOLO inference budget. Resumes automatically once the run
  // reaches PHASE_SUCCEEDED or PHASE_FAILED. Set false to make this node
  // ignore calibration status entirely and keep running regardless (e.g.
  // for isolated testing/debugging depth_perception on its own).
  bool pause_while_calibration = true;

  // Number of most-recent valid back-projected samples kept per tracked
  // instance (cup_holder, or each hole_1..hole_4) before the oldest is
  // dropped — see RollingWindow's doc comment. Same "average over several
  // samples to cancel per-frame noise" idea calibration_broadcaster_node
  // already uses (its own num_samples param), applied here per-instance
  // instead of once for a whole calibration run.
  int rolling_window_size = 15;

  // Live-lab testing (2026-07-27) found the rolling window's median alone
  // insufficient: YOLO intermittently returns NO detection at all for a
  // given hole/cup_holder on many individual frames (not noisy-but-present
  // — genuinely absent that frame), which a bigger window/more samples
  // can't fix, since there's nothing to average when no sample arrives at
  // all — confirmed live by the user cranking rolling_window_size up to
  // 100000 with zero improvement. The fix: since holes/cup_holder are
  // physically FIXED (unlike the arm-mounted marker), this node now holds
  // the last known-good position across any gap, and only updates it when
  // a new valid sample is far enough away to be a genuine change rather
  // than per-frame jitter — see RollingWindow::last_stable's doc comment.
  // Distance (meters) a new sample must exceed from the current
  // last_stable position before it's accepted as a real position change;
  // anything closer is treated as noise around an already-known-good
  // position and doesn't move last_stable at all.
  double stable_drift_threshold_m = 0.10;

  // Topic this node publishes visual_calibration_msgs/StablePositionArray
  // on — the continuous, gap-free "held position" stream for every
  // tracked instance ever successfully detected, camera-frame position
  // plus its reprojected pixel coordinates (see StablePosition.msg's own
  // doc comment). Consumers: yolo_marker_bridge_node's overlay image
  // (drawing a stabilized dot alongside its own raw per-frame boxes), and
  // this node's own broadcastInstanceTfs() (see below).
  std::string stable_positions_topic = "/depth_perception/stable_positions";

  // --- cup_holder/hole TF broadcasting (2026-07-29) ---
  // TF frame at the base of the calibrated chain — same frame
  // calibration_broadcaster_node's known_chain_frame names ("base_link" in
  // both sim and real, confirmed against calibration_broadcaster_{sim,real}.yaml).
  std::string known_chain_frame = "base_link";

  // Appended to the incoming StablePositionArray/detections_2d's own
  // header.frame_id to form the calibrated camera frame name to look up —
  // e.g. "D415_color_optical_frame" (real) becomes
  // "D415_color_optical_frame_calibrated". MUST match
  // calibration_broadcaster_node's own broadcast_frame_suffix exactly
  // (same convention, deliberately not hardcoding a per-env frame name
  // here — the incoming message's own frame_id already tells us which
  // camera frame is in play, sim or real, so appending this suffix is
  // sufficient without a separate sim/real frame-name split like the web
  // app's markerFrames.ts needed).
  std::string broadcast_frame_suffix = "_calibrated";

  // When true (default), broadcastInstanceTfs() looks up the calibrated
  // camera TF and publishes known_chain_frame -> cup_holder/hole_1..4 as
  // live (non-static) TF frames, chaining each instance's held camera-
  // frame position through the calibration result. Set false to disable
  // (e.g. before any calibration has ever run — the lookup would simply
  // fail/skip silently every time anyway, but this avoids the repeated
  // failed-lookup log noise in that case).
  bool broadcast_instance_tfs = true;

  // --- Depth-view overlay (2026-08-04) ---
  // A SEPARATE overlay stream from yolo_marker_bridge_node's own
  // /aruco_perception/overlay_image (which always uses the COLOR image as
  // its base) — this one uses the colorized DEPTH image as its base
  // instead, with the same detection centroids (and the actual sampled
  // patch circle backProjectDetection used) drawn on top. Added so a human
  // can directly compare "where YOLO says a hole/cup_holder is" against
  // "what the depth camera actually sees there" — the diagnostic the
  // real-robot hole/cup_holder TF placement investigation needed (see this
  // fix's own plan doc). Mirrors aruco_detector_node's own
  // publish_overlay_image/overlay_image_topic param pair exactly, same
  // image_transport::create_publisher pattern.
  bool publish_depth_overlay_image = true;
  std::string depth_overlay_image_topic = "/depth_perception/overlay_image";
};

// instance_tf_z_offset_m (2026-07-30) is deliberately NOT a field on
// DepthPerceptionConfig above, unlike every other setting in this struct —
// read LIVE via get_parameter() at the top of broadcastInstanceTfs() every
// call instead, so it can be tuned via `ros2 param set` with no node
// restart. Same "live, not cached" convention calibration_broadcaster_node's
// use_clustering_average already established for exactly this reason — see
// that node's own comments contrasting live vs. cached params. Added
// straight to cup_holder's broadcast translation.z; hole_1..hole_4 inherit
// it automatically since they're parented to cup_holder, not
// known_chain_frame, directly (see broadcastInstanceTfs's own comment on
// that re-anchoring). Default 0.0 = no change from the raw detected
// position.
//
// Two distinct, independently-valid uses, per environment:
// - Sim (2026-08-02): corrects a real depth-perception artifact — the raw
//   back-projected TF lands slightly BELOW the holder/hole's true surface
//   (effectively "buried" a little into the cavity, not sitting on top of
//   it), so a small positive lift (e.g. 0.05m) places the broadcast TF at
//   the actual physical surface. This is a genuine position correction,
//   not a reachability workaround, on sim.
// - Real (2026-07-30, original motivation): was added as a fast,
//   explicitly-requested reachability workaround under time pressure —
//   real's cup_holder HORIZONTAL (X/Y) distance from base_link alone was
//   measured at ~0.64m, past the UR3e's ~0.5m datasheet reach, so a
//   Z-only offset alone cannot make that position actually reachable
//   (distance there is dominated by the horizontal leg, which this offset
//   never touches) — kept OFF (0.0) on real as of 2026-07-30 while the
//   accuracy investigation in
//   _errors/finetuning_modification_floor_plan.txt is ongoing, since it
//   masks rather than fixes bad TF placement; re-enable there only if
//   reachability becomes the immediate blocker again, independent of the
//   accuracy work.
// Set to 0.0 (or leave undeclared) to fully undo either use.

// instance_tf_xy_offset_m (2026-08-03) — same "live, not cached" convention
// as instance_tf_z_offset_m directly above (read via get_parameter_or every
// broadcastInstanceTfs() call, no restart needed), but for X/Y instead of
// Z. Added straight to cup_holder's broadcast translation.x/y ONLY;
// hole_1..hole_4 inherit any shift automatically since they're parented to
// cup_holder, not known_chain_frame, directly (same reasoning as
// instance_tf_z_offset_m — no separate per-hole X/Y offset is needed).
//
// Motivation: cup_holder's 2D pixel centroid (cv::fitEllipse on the rim
// contour, sim's classical detector only — see cup_holder_detector_node's
// own refineCupHolderCircle doc comment) is pulled slightly toward the rim
// geometry rather than landing exactly on the true visual/physical center,
// confirmed via a live screenshot showing the fitted TF offset from center
// (2026-08-02). This offset is a manual, tunable compensation for that
// bias, NOT a change to the CV algorithm itself.
// Default (0.0, 0.0) — a 2-element array, [x_offset_m, y_offset_m] — no
// change from today's raw detected position until tuned.
// NOT yet confirmed present on real (real's cup_holder centroid comes from
// YOLO, a completely different detector with no fitEllipse step at all —
// see the fix's plan doc's Context section) — left at (0.0, 0.0) there
// until a real-side measurement actually shows a comparable bias.

/*
 * One detection's computed 3D position, expressed in the camera's own
 * optical frame (the same frame convention aruco_detector_node's
 * marker_pose already uses) — meters, right-handed, +Z out of the lens.
 * Position only for now (Step 2) — no orientation, no averaging across
 * frames, no publishing yet; this struct exists purely so
 * backProjectDetection()'s result can be logged and, in a later step,
 * handed to whatever does the multi-frame filtering.
 */
struct BackProjectedPoint
{
  bool valid = false;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

/*
 * One detection's data as needed by publishDepthOverlayImage (2026-08-04)
 * — collected alongside detections2dCallback's own main back-projection
 * loop so the overlay draws the EXACT patch_half_px backProjectDetection
 * actually used for that detection, rather than a second, possibly
 * out-of-sync computation of the same radius-scaling logic.
 */
struct OverlayDetection
{
  std::string class_name;
  int32_t hole_number = 0;
  double cx = 0.0;
  double cy = 0.0;
  int patch_half_px = 0;
  bool valid = false;
  double z = 0.0;
};

/*
 * Identifies one physical thing being tracked across frames: "cup_holder"
 * (hole_number always 0, since only one ever exists in frame — see
 * Detection2D.msg's own doc comment) or one specific hole ("hole",
 * hole_number 1-4, the fixed quadrant label yolo_marker_bridge_node.py
 * assigns). Used as a std::map key so each physical instance gets its own
 * independent rolling window — averaging hole_1's samples together with
 * hole_2's would silently produce a meaningless midpoint between two
 * different holes.
 */
struct TrackedInstanceKey
{
  std::string class_name;
  int32_t hole_number = 0;

  bool operator<(const TrackedInstanceKey & other) const
  {
    if (class_name != other.class_name) {
      return class_name < other.class_name;
    }
    return hole_number < other.hole_number;
  }
};

/*
 * A capped history of one tracked instance's recent valid back-projected
 * points (see DepthPerceptionConfig::rolling_window_size), plus the median
 * position computed from whatever's currently in the buffer. Median (not
 * mean) for the same reason backProjectDetection() already uses a median
 * within a single frame's depth patch: robust against a minority of
 * outlier samples (e.g. one frame's detection briefly landing partly on
 * the tray surface instead of the hole itself).
 *
 * Deliberately still camera-frame only, still no orientation, still not
 * published — this struct only removes FRAME-TO-FRAME noise. Chaining
 * through the calibrated camera->base_link TF is a separate, later step
 * (blocked on a live calibration run existing — see class doc comment).
 */
struct RollingWindow
{
  std::deque<BackProjectedPoint> samples;

  // The instance's held, "known-good" position (see
  // DepthPerceptionConfig::stable_drift_threshold_m's doc comment for the
  // full rationale) — updated only when a new median differs from this by
  // more than stable_drift_threshold_m, and left completely untouched on
  // any frame where no new sample arrives at all (samples/push() aren't
  // even called that frame). This is what actually fixes "flicker": a gap
  // in detection no longer means a gap in the reported position, since
  // last_stable simply keeps its last value until a REAL change is seen.
  BackProjectedPoint last_stable;

  // Whether the MOST RECENT call to updateLastStable() actually changed
  // last_stable (a genuine drift) vs. left it untouched (noise) — set by
  // updateLastStable(), read by DepthPerceptionNode::publishStablePositions()
  // for StablePosition.msg's own `drifted` field. Reflects the last time
  // this instance's state was actually updated, not necessarily this
  // exact publish cycle — an instance absent from the triggering frame's
  // detections_2d simply republishes this unchanged, per
  // StablePositionArray.msg's "continuous stream" design.
  bool last_update_drifted = false;

  // Wall-clock time of the most recent ACTUAL detection for this instance
  // (2026-08-06) — set every time updateRollingWindow() is called for this
  // instance, regardless of whether the new sample changed last_stable or
  // not (i.e. a genuine "still being seen" heartbeat, distinct from
  // last_update_drifted's "position actually changed" meaning). Needed
  // because last_stable itself has NO concept of staleness by design (see
  // its own doc comment) — an instance that stops being detected entirely
  // (e.g. a hole becomes occupied/obstructed) would otherwise keep
  // broadcasting its last known TF forever, with nothing to signal it
  // should stop. See DepthPerceptionConfig::instance_stale_timeout_s and
  // broadcastInstanceTfs()'s own doc comment for how this is used.
  // Default-constructed (epoch/zero) until the first real detection.
  rclcpp::Time last_seen;

  void push(const BackProjectedPoint & point, size_t max_size)
  {
    samples.push_back(point);
    while (samples.size() > max_size) {
      samples.pop_front();
    }
  }

  // Median x/y/z independently across the buffer (not a true 3D
  // median/geometric median — each axis is reduced on its own, matching
  // backProjectDetection()'s existing per-axis approach and avoiding the
  // extra complexity of a proper multivariate median for what's still a
  // debug/log-only checkpoint).
  BackProjectedPoint median() const;

  // Straight-line 3D distance between two points — used by
  // updateLastStable() below to decide whether a new median is a genuine
  // position change or just noise around an already-known-good position.
  static double distance(const BackProjectedPoint & a, const BackProjectedPoint & b);

  // Called after push() with the buffer's freshly recomputed median.
  // First call ever for this instance: last_stable is simply seeded with
  // it (nothing to compare against yet). Afterwards: last_stable is only
  // replaced if the new median is farther than
  // DepthPerceptionConfig::stable_drift_threshold_m away from it — a real
  // physical change (e.g. the operator moved the cupholder), not per-frame
  // jitter around a fixed physical object. Returns true if last_stable was
  // actually updated (purely for logging "drift detected" vs "held").
  bool updateLastStable(const BackProjectedPoint & new_median, double drift_threshold_m);
};

/*
 * Step 2/3 node: on top of Step 1's plumbing (RGB/depth/camera_info
 * subscriptions, still present), this node subscribes to
 * yolo_marker_bridge_node's Detection2DArray and, for every cup_holder/
 * hole detection in each incoming array, samples the depth image around
 * that detection's pixel centroid and back-projects it into a 3D point in
 * the camera's optical frame.
 *
 * Runs continuously from startup — it does NOT wait for a calibration run
 * to exist first (sensing in the camera's own frame has no dependency on
 * calibration at all; only the later camera->base_link chaining step
 * will). It DOES pause itself while a ~/auto_calibrate run is actively in
 * progress (see pause_while_calibration/autoCalibrateStatusCallback), so
 * it isn't competing with marker detection for the same YOLO inference
 * budget during exactly the window where marker detection speed/
 * reliability matters most.
 *
 * Deliberately NOT doing yet (left for a later step): chaining through the
 * calibrated camera->base_link TF, orientation estimation, or publishing
 * any topic — this node still only logs its results. See
 * Detection2D.msg's own doc comment for why multi-frame filtering is
 * needed at all (single-frame depth is noisy near a cavity rim).
 */
class DepthPerceptionNode : public rclcpp::Node
{
public:
  DepthPerceptionNode();

private:
  // Reads all topic names/tuning values from this node's declared
  // parameters. Requires the node to be started with a parameter file
  // providing every field (e.g. via
  // automatically_declare_parameters_from_overrides).
  DepthPerceptionConfig loadConfigFromParams() const;

  // Logs the RGB frame's dimensions/encoding once per throttle period.
  void rgbImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  // Stores the latest depth frame (converted to a cv::Mat of meters) for
  // detections2dCallback to read. Depth images use a different encoding
  // (e.g. 32FC1: one float per pixel) than the RGB image, so this is a
  // separate callback rather than reusing rgbImageCallback.
  void depthImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  // Logs that camera intrinsics were received — only once, since
  // camera_info is republished at a steady rate and doesn't change
  // between frames.
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);

  // For every cup_holder/hole entry in the incoming array, back-projects
  // its (cx, cy) using the most recently stored depth frame + camera
  // intrinsics, and logs the resulting 3D point. Does nothing (logs a
  // throttled "waiting" message instead) until both a depth frame and
  // camera_info have been received at least once, or while paused (see
  // autoCalibrateStatusCallback). aruco_marker entries are ignored here —
  // that detection already has its own, more precise, solvePnP-based 3D
  // pose published separately on marker_pose.
  void detections2dCallback(
    const visual_calibration_msgs::msg::Detection2DArray::ConstSharedPtr & msg);

  // Tracks calibration_orchestrator_node's ~/auto_calibrate progress
  // purely to drive calibration_paused_ (see
  // DepthPerceptionConfig::pause_while_calibration's doc comment) — this
  // node has no other interest in calibration's outcome. A no-op if
  // config_.pause_while_calibration is false.
  void autoCalibrateStatusCallback(
    const visual_calibration_msgs::msg::AutoCalibrateStatus::ConstSharedPtr & msg);

  // Reads a depth value (median OR max, see use_max_depth below) from a
  // small square patch of latest_depth_ centered at (cx, cy), then converts
  // the pixel + depth into a 3D point via the standard pinhole
  // back-projection:
  //   X = (u - cx_intrinsic) * depth / fx
  //   Y = (v - cy_intrinsic) * depth / fy
  //   Z = depth
  // Returns BackProjectedPoint::valid = false if camera_info/depth aren't
  // ready yet, the pixel falls outside the depth image, or every pixel in
  // the sampled patch is invalid (e.g. NaN/zero, meaning "no depth
  // return" — a real possibility right at a cavity's rim).
  //
  // bbox (2026-08-03, [x1, y1, x2, y2] pixels, straight from
  // Detection2D.msg — see that message's own doc comment) is used to
  // derive this specific detection's own radius
  // (max(bbox[2]-bbox[0], bbox[3]-bbox[1]) / 2.0 — max, not assumed square,
  // since real's YOLO-published bbox is a true axis-aligned box, not a
  // synthesized square the way sim's classical detector's is) and scale
  // the patch half-size sampled around (cx, cy) to
  // clamp(depth_patch_radius_scale_factor * radius_px,
  // depth_patch_min_half_size_px, depth_patch_half_size_px) instead of
  // unconditionally using the fixed depth_patch_half_size_px for every
  // detection regardless of size.
  //
  // Motivation (patch SIZE, 2026-08-03): a small hole's radius can be close
  // to or smaller than the fixed patch half-size, so that fixed patch
  // straddles the cavity's inner wall — some sampled pixels land on the RIM
  // (shallower depth) instead of the true FLOOR (farther depth), biasing
  // the median Z toward the rim even though (cx, cy) itself is the correct
  // centroid (confirmed via full pipeline trace, 2026-08-02/03 — the 2D
  // centroid is NOT the bug for holes, this patch-averaging step is).
  // Scaling the patch to the hole's own radius keeps it inside the true
  // floor. This alone does NOT fully solve the rim/wall bias, though — see
  // use_max_depth below for the remaining case it doesn't cover.
  //
  // use_max_depth (2026-08-04) — a SEPARATE, ANGLE-related bias from the
  // patch-SIZE fix above: even a patch correctly sized to stay inside a
  // hole can be centered on a pixel whose straight-line ray to the camera
  // grazes the cavity's near wall before ever reaching the true floor —
  // an oblique viewing-angle problem, not a patch-size problem. A
  // wall-grazing ray returns a SHORTER depth than one that reaches the true
  // (farther) floor, so MEDIAN-of-patch can still pick a wall-biased value
  // even with an appropriately-sized patch, if enough of the sampled pixels
  // are also seeing wall rather than floor. When true, this function uses
  // the FARTHEST (max) valid depth in the patch instead of the median — the
  // best available proxy, from a single small patch, for "the one sample
  // that actually made it to the true floor." Callers should pass true for
  // "hole" detections (a real cavity, where the wall-grazing effect
  // applies) and false for "cup_holder" (a raised, roughly flat rim/surface
  // with no equivalent cavity to graze past — median stays the
  // noise-robust choice there, since max-of-patch on a flat surface risks
  // picking up a stray outlier spike instead of a real signal).
  //
  // out_radius_px/out_patch_half_px (2026-08-03) report exactly what this
  // call computed/used, purely so detections2dCallback can log them for
  // diagnostic purposes (see this class's own doc comment on the fix's
  // logging-first-then-tune approach) — this function itself stays a
  // logically-const geometry computation with no logging inside it
  // (RCLCPP_INFO_THROTTLE cannot be called from a const method: it needs
  // Clock::now(), which is non-const, and get_clock() const only returns a
  // ConstSharedPtr). Set on every call, including early-return paths (no
  // depth frame yet, pixel outside the image, empty patch), so the
  // caller's log always has a real value even when back-projection fails.
  BackProjectedPoint backProjectDetection(
    double cx, double cy, const std::array<double, 4> & bbox,
    bool use_max_depth, double & out_radius_px, int & out_patch_half_px) const;

  // Pushes `point` into the rolling window for (class_name, hole_number)
  // — creating a new, empty window on first sight of that instance — then
  // recomputes the window's median and feeds it through
  // RollingWindow::updateLastStable(), returning the resulting last_stable
  // point (NOT the raw median — see that method's doc comment for why:
  // holding a fixed, physically-static object's last known-good position
  // across detection gaps is what actually fixes the flicker symptom this
  // was built for). Writes the window's current sample count to
  // out_window_size and whether last_stable just changed to out_drifted
  // (both purely for logging in detections2dCallback). Only ever called
  // with valid points (see detections2dCallback); an instance that briefly
  // fails to back-project just doesn't get a new sample that frame — its
  // existing window/last_stable is left completely as-is, not reset.
  BackProjectedPoint updateRollingWindow(
    const std::string & class_name, int32_t hole_number, const BackProjectedPoint & point,
    size_t & out_window_size, bool & out_drifted);

  // Publishes one StablePositionArray containing every tracked instance's
  // (cup_holder, hole_1..hole_4) held last_stable position — called once
  // per detections2dCallback invocation, after that frame's state updates
  // are applied, so every publish reflects the latest known-good state
  // regardless of what (if anything) arrived in this specific frame. See
  // StablePositionArray.msg's own doc comment for why this is
  // "every instance ever seen, every callback," not just this frame's
  // arrivals — that continuity is the whole point of this topic existing
  // alongside the raw, gap-prone detections_2d stream.
  void publishStablePositions(const std_msgs::msg::Header & header);

  // Standard pinhole reprojection — the inverse of backProjectDetection():
  // given a 3D point already known to be in the camera's optical frame,
  // returns the 2D pixel coordinates it would appear at. Used so
  // consumers that only want to DRAW a stable position (e.g.
  // yolo_marker_bridge_node's overlay) never need their own camera-frame
  // math — see StablePosition.msg's px/py fields.
  void reprojectToPixels(const BackProjectedPoint & point, double & out_px, double & out_py) const;

  // Broadcasts config_.known_chain_frame -> "cup_holder"/"hole_1".."hole_4"
  // (2026-07-29) for every tracked instance in rolling_windows_ whose
  // last_stable is valid, chaining each instance's camera-frame held
  // position through the calibrated camera TF (config_.known_chain_frame
  // -> header.frame_id + config_.broadcast_frame_suffix, looked up fresh
  // via tf_buffer_ every call — calibration_broadcaster_node broadcasts
  // that frame once as a LATCHED static TF after a ~/calibrate run
  // completes, so a lookup here works immediately whether that broadcast
  // happened before or after this node started). No-op (logs a throttled
  // warning once) if that lookup fails — e.g. no ~/calibrate run has ever
  // completed yet in this session — same "silently does nothing useful
  // until its dependency exists" pattern this project's other calibration-
  // dependent consumers already use (see depth-perception agent's own
  // doc comment on this exact dependency). A no-op entirely if
  // config_.broadcast_instance_tfs is false. Called from
  // publishStablePositions() (same header/instances, one call site).
  void broadcastInstanceTfs(const std_msgs::msg::Header & header);

  // Depth-view overlay (2026-08-04) — see DepthPerceptionConfig::
  // publish_depth_overlay_image's own doc comment for the full rationale.
  // Colorizes the CURRENT latest_depth_ (cv::normalize + applyColorMap,
  // COLORMAP_JET — standard depth-visualization convention) into a BGR
  // image, draws every entry in `detections` (this callback's own
  // msg->detections, so the overlay always matches exactly what this
  // frame's back-projection loop actually processed) as a centroid dot +
  // the ACTUAL sampled patch circle (patch_half_px, per-detection — the
  // most direct way to see exactly what backProjectDetection looked at),
  // and publishes the result on depth_overlay_image_pub_. No-op if
  // config_.publish_depth_overlay_image is false or latest_depth_ is still
  // empty (no depth frame received yet). Called once per
  // detections2dCallback invocation, after that frame's own
  // back-projection loop (so it has each detection's already-computed
  // patch_half_px available, not a second independent computation).
  void publishDepthOverlayImage(
    const std_msgs::msg::Header & header,
    const std::vector<OverlayDetection> & detections);

  DepthPerceptionConfig config_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster instance_tf_broadcaster_;

  // One rolling window per physical instance seen so far (cup_holder;
  // hole_1..hole_4) — see TrackedInstanceKey/RollingWindow's own doc
  // comments. Grows to at most 5 entries in this project's actual use
  // case (1 cup_holder + 4 holes); not guarded by a mutex since it's only
  // ever touched from detections2dCallback, which rclcpp does not invoke
  // concurrently with itself for a single subscription.
  std::map<TrackedInstanceKey, RollingWindow> rolling_windows_;

  image_transport::Subscriber rgb_image_sub_;
  image_transport::Subscriber depth_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<visual_calibration_msgs::msg::Detection2DArray>::SharedPtr
    detections_2d_sub_;
  rclcpp::Subscription<visual_calibration_msgs::msg::AutoCalibrateStatus>::SharedPtr
    auto_calibrate_status_sub_;
  rclcpp::Publisher<visual_calibration_msgs::msg::StablePositionArray>::SharedPtr
    stable_positions_pub_;
  // Depth-view overlay (2026-08-04) — see DepthPerceptionConfig::
  // publish_depth_overlay_image's own doc comment. Only created if that
  // param is true, same "unset if the feature is off" convention as
  // aruco_detector_node's own overlay_image_pub_.
  image_transport::Publisher depth_overlay_image_pub_;

  bool camera_info_received_ = false;

  // True while an ~/auto_calibrate run is in progress (phase ==
  // PHASE_RUNNING) — set/cleared by autoCalibrateStatusCallback, read by
  // detections2dCallback. Starts false: this node assumes no calibration
  // is running until told otherwise, matching the "run continuously from
  // startup" behavior decided for this node — see class doc comment.
  bool calibration_paused_ = false;

  // Pinhole intrinsics captured once from camera_info_sub_'s first
  // message — fx/fy (focal lengths in pixels), cx/cy (optical center in
  // pixels). Same source convention as aruco_detector_node's
  // camera_matrix_ (msg->k[0..8], row-major 3x3).
  double fx_ = 0.0;
  double fy_ = 0.0;
  double cx_intrinsic_ = 0.0;
  double cy_intrinsic_ = 0.0;

  // Latest depth frame, converted to CV_32F meters (config_.depth_scale_to_meters
  // already applied) — guarded by depth_mutex_ since it's written from
  // depthImageCallback and read from detections2dCallback, which may run
  // on different callback invocations under a multi-threaded executor.
  // mutable: backProjectDetection() is logically const (it doesn't change
  // this node's observable state) but still needs to lock this mutex to
  // safely read latest_depth_.
  mutable std::mutex depth_mutex_;
  cv::Mat latest_depth_;
  bool depth_received_ = false;
};

}  // namespace depth_perception

#endif  // DEPTH_PERCEPTION__DEPTH_PERCEPTION_NODE_HPP_
