#ifndef DEPTH_PERCEPTION__DEPTH_PERCEPTION_NODE_HPP_
#define DEPTH_PERCEPTION__DEPTH_PERCEPTION_NODE_HPP_

#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
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
  // pixel.
  int depth_patch_half_size_px = 2;

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
  // eventually a camera->base_link TF broadcaster once calibration exists
  // to chain through.
  std::string stable_positions_topic = "/depth_perception/stable_positions";
};

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

  // Reads a robust (median) depth value from a small square patch of
  // latest_depth_ centered at (cx, cy), then converts the pixel + depth
  // into a 3D point via the standard pinhole back-projection:
  //   X = (u - cx_intrinsic) * depth / fx
  //   Y = (v - cy_intrinsic) * depth / fy
  //   Z = depth
  // Returns BackProjectedPoint::valid = false if camera_info/depth aren't
  // ready yet, the pixel falls outside the depth image, or every pixel in
  // the sampled patch is invalid (e.g. NaN/zero, meaning "no depth
  // return" — a real possibility right at a cavity's rim).
  BackProjectedPoint backProjectDetection(double cx, double cy) const;

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

  DepthPerceptionConfig config_;

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
