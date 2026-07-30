#include "depth_perception/depth_perception_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace depth_perception
{

DepthPerceptionNode::DepthPerceptionNode()
: Node(
    "depth_perception_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
  config_(loadConfigFromParams()),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_),
  instance_tf_broadcaster_(this)
{
  rgb_image_sub_ = image_transport::create_subscription(
    this, config_.rgb_image_topic,
    std::bind(&DepthPerceptionNode::rgbImageCallback, this, std::placeholders::_1),
    "raw");

  depth_image_sub_ = image_transport::create_subscription(
    this, config_.depth_image_topic,
    std::bind(&DepthPerceptionNode::depthImageCallback, this, std::placeholders::_1),
    "raw");

  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    config_.camera_info_topic, rclcpp::SensorDataQoS(),
    std::bind(&DepthPerceptionNode::cameraInfoCallback, this, std::placeholders::_1));

  detections_2d_sub_ = create_subscription<visual_calibration_msgs::msg::Detection2DArray>(
    config_.detections_2d_topic, rclcpp::QoS(10),
    std::bind(&DepthPerceptionNode::detections2dCallback, this, std::placeholders::_1));

  if (config_.pause_while_calibration) {
    auto_calibrate_status_sub_ =
      create_subscription<visual_calibration_msgs::msg::AutoCalibrateStatus>(
      config_.auto_calibrate_status_topic, rclcpp::QoS(10).reliable(),
      std::bind(&DepthPerceptionNode::autoCalibrateStatusCallback, this, std::placeholders::_1));
  }

  // Same QoS (plain reliable, depth-10) as detections_2d_pub/marker_pose —
  // a continuous derived stream, not a one-shot/latched value. See
  // StablePositionArray.msg's own doc comment.
  stable_positions_pub_ = create_publisher<visual_calibration_msgs::msg::StablePositionArray>(
    config_.stable_positions_topic, rclcpp::QoS(10));

  RCLCPP_INFO(
    get_logger(),
    "depth_perception_node ready (rgb: '%s', depth: '%s', camera_info: '%s', "
    "detections_2d: '%s', pause_while_calibration: %s, stable_positions: '%s')",
    config_.rgb_image_topic.c_str(), config_.depth_image_topic.c_str(),
    config_.camera_info_topic.c_str(), config_.detections_2d_topic.c_str(),
    config_.pause_while_calibration ? "true" : "false",
    config_.stable_positions_topic.c_str());
}

void DepthPerceptionNode::rgbImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  const cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 5000,
    "Received RGB frame: %dx%d, encoding '%s'",
    cv_ptr->image.cols, cv_ptr->image.rows, msg->encoding.c_str());
}

void DepthPerceptionNode::depthImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  // toCvCopy (not toCvShare) is required here, unlike rgbImageCallback:
  // we convert to a fixed CV_32FC1 representation below regardless of the
  // incoming encoding, and toCvShare cannot perform a real pixel-format
  // conversion (see aruco_perception's error-mitigation notes on this
  // exact toCvShare-vs-toCvCopy distinction).
  const cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 5000,
    "Received depth frame: %dx%d, encoding '%s'",
    cv_ptr->image.cols, cv_ptr->image.rows, msg->encoding.c_str());

  // Store a CV_32F-meters copy for detections2dCallback to read. Scaling
  // by config_.depth_scale_to_meters here (once, at storage time) means
  // backProjectDetection() never needs to know or care what the source
  // encoding/units were — see DepthPerceptionConfig::depth_scale_to_meters's
  // own doc comment for why this defaults to 1.0 (32FC1 sim depth is
  // already meters) but exists as a parameter regardless.
  cv::Mat depth_meters;
  if (msg->encoding == "32FC1") {
    depth_meters = cv_ptr->image * config_.depth_scale_to_meters;
  } else {
    // 16UC1 (millimeters) is the other encoding real depth cameras
    // commonly publish — convert to float first so the multiply below
    // doesn't saturate/truncate as an integer type.
    cv_ptr->image.convertTo(depth_meters, CV_32F, config_.depth_scale_to_meters);
  }

  {
    std::lock_guard<std::mutex> lock(depth_mutex_);
    latest_depth_ = depth_meters;
    depth_received_ = true;
  }
}

void DepthPerceptionNode::cameraInfoCallback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
{
  if (camera_info_received_) {
    return;
  }

  // Same source convention as aruco_detector_node's camera_matrix_:
  // msg->k is a row-major 3x3 matrix [fx 0 cx; 0 fy cy; 0 0 1].
  fx_ = msg->k[0];
  fy_ = msg->k[4];
  cx_intrinsic_ = msg->k[2];
  cy_intrinsic_ = msg->k[5];

  camera_info_received_ = true;

  RCLCPP_INFO(
    get_logger(), "Received camera_info: %ux%u, distortion model '%s'",
    msg->width, msg->height, msg->distortion_model.c_str());
}

void DepthPerceptionNode::detections2dCallback(
  const visual_calibration_msgs::msg::Detection2DArray::ConstSharedPtr & msg)
{
  if (calibration_paused_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Paused while ~/auto_calibrate is running (pause_while_calibration=true) — "
      "not processing detections_2d.");
    return;
  }

  if (!camera_info_received_ || !depth_received_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Got detections_2d but still waiting on camera_info/depth before back-projecting.");
    return;
  }

  // Built up as one multi-line string and logged ONCE per callback
  // (throttled), rather than calling RCLCPP_INFO_THROTTLE once per
  // detection inside the loop below. RCLCPP_INFO_THROTTLE's "have I
  // logged recently" bucket is keyed by call site (source line), not by
  // any argument — so if every detection in a frame shared one throttled
  // call, they'd all contend for the SAME bucket and only the first
  // detection in msg->detections (always cup_holder, per
  // yolo_marker_bridge_node's publish order) would ever get through.
  // Confirmed live 2026-07-27: exactly this symptom (hole detections
  // present in /aruco_perception/detections_2d but never logged here).
  std::string log_lines;

  for (const auto & detection : msg->detections) {
    // aruco_marker already gets a precise solvePnP-based 3D pose on
    // marker_pose — this node's job is cup_holder/hole only, per
    // Detection2D.msg's own doc comment.
    if (detection.class_name != "cup_holder" && detection.class_name != "hole") {
      continue;
    }

    const BackProjectedPoint point = backProjectDetection(detection.cx, detection.cy);

    // hole_number is only meaningful for "hole" (see Detection2D.msg's own
    // doc comment) — cup_holder always uses 0, matching the message's own
    // "unset for non-hole classes" convention, so TrackedInstanceKey never
    // has to special-case class_name itself.
    const int32_t hole_number = (detection.class_name == "hole") ? detection.hole_number : 0;

    char line[256];
    if (point.valid) {
      size_t window_size = 0;
      bool drifted = false;
      const BackProjectedPoint stable =
        updateRollingWindow(detection.class_name, hole_number, point, window_size, drifted);
      char instance_desc[32];
      if (detection.class_name == "hole") {
        std::snprintf(instance_desc, sizeof(instance_desc), "hole_%d", hole_number);
      } else {
        std::snprintf(instance_desc, sizeof(instance_desc), "%s", detection.class_name.c_str());
      }
      std::snprintf(
        line, sizeof(line),
        "\n  %s: frame(x=%.3f, y=%.3f, z=%.3f) stable(x=%.3f, y=%.3f, z=%.3f) "
        "m, n=%zu/%d, %s, confidence=%.2f",
        instance_desc, point.x, point.y, point.z, stable.x, stable.y, stable.z,
        window_size, config_.rolling_window_size, drifted ? "DRIFT" : "held",
        detection.confidence);
    } else {
      // A single frame's failed back-projection does NOT touch that
      // instance's rolling window — see updateRollingWindow's doc
      // comment — so its existing stable estimate (if any) is left
      // alone rather than being reported as missing here. NOTE: this
      // "no detection at all this frame" case (a hole simply absent from
      // msg->detections) never reaches this loop body in the first place
      // — nothing is pushed into that instance's rolling window on such a
      // frame, so RollingWindow::samples never contains a "missing" entry
      // and median()/last_stable's drift check only ever sees real,
      // valid back-projected points — confirmed 2026-07-27, this was
      // already correct, not something this change needed to fix.
      std::snprintf(
        line, sizeof(line),
        "\n  %s at pixel (%.1f, %.1f): no valid depth in the sampled patch",
        detection.class_name.c_str(), detection.cx, detection.cy);
    }
    log_lines += line;
  }

  if (!log_lines.empty()) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000, "detections_2d back-projected:%s", log_lines.c_str());
  }

  // Published every callback regardless of what (if anything) arrived
  // this specific frame — see StablePositionArray.msg's own doc comment
  // for why this is the continuous stream, unlike the log above.
  publishStablePositions(msg->header);
}

void DepthPerceptionNode::publishStablePositions(const std_msgs::msg::Header & header)
{
  visual_calibration_msgs::msg::StablePositionArray array_msg;
  array_msg.header = header;

  for (const auto & [key, window] : rolling_windows_) {
    if (!window.last_stable.valid) {
      // Entry exists in rolling_windows_ (created by updateRollingWindow)
      // but never actually got a valid back-projection yet — shouldn't
      // normally happen, kept as a defensive guard only.
      continue;
    }

    visual_calibration_msgs::msg::StablePosition position;
    position.class_name = key.class_name;
    position.hole_number = key.hole_number;
    position.x = window.last_stable.x;
    position.y = window.last_stable.y;
    position.z = window.last_stable.z;
    reprojectToPixels(window.last_stable, position.px, position.py);
    // Reflects whether the MOST RECENT call to updateLastStable() for
    // this instance actually changed last_stable — not necessarily
    // something that happened this exact callback, since an instance
    // absent from this frame's detections_2d simply republishes its
    // already-known state/drifted flag unchanged. See RollingWindow::
    // last_update_drifted's own doc comment.
    position.drifted = window.last_update_drifted;
    position.sample_count = static_cast<int32_t>(window.samples.size());
    array_msg.positions.push_back(position);
  }

  stable_positions_pub_->publish(array_msg);

  if (config_.broadcast_instance_tfs) {
    broadcastInstanceTfs(header);
  }
}

void DepthPerceptionNode::broadcastInstanceTfs(const std_msgs::msg::Header & header)
{
  const std::string calibrated_camera_frame = header.frame_id + config_.broadcast_frame_suffix;

  geometry_msgs::msg::TransformStamped known_to_camera_tf;
  try {
    known_to_camera_tf = tf_buffer_.lookupTransform(
      config_.known_chain_frame, calibrated_camera_frame, tf2::TimePointZero,
      tf2::durationFromSec(0.5));
  } catch (const tf2::TransformException & ex) {
    // Expected/routine until a ~/calibrate run has completed at least
    // once this session (calibration_broadcaster_node only ever
    // broadcasts calibrated_camera_frame after a successful run) — not an
    // error worth spamming every callback for.
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "broadcastInstanceTfs: '%s' -> '%s' not available yet (%s) — has a ~/calibrate run "
      "completed this session?",
      config_.known_chain_frame.c_str(), calibrated_camera_frame.c_str(), ex.what());
    return;
  }

  tf2::Transform known_to_camera;
  tf2::fromMsg(known_to_camera_tf.transform, known_to_camera);

  // cup_holder anchors the 4 holes (2026-07-30) — find its current
  // last_stable first so hole TFs below can be expressed relative to it
  // (cup_holder -> hole_N) instead of each hole independently re-deriving
  // its own base_link -> hole_N chain. All holes move together with the
  // holder as one rigid unit in reality, so anchoring to the holder's own
  // tracked centroid — rather than base_link — is both the physically
  // correct parent and immune to the holder's own position noise being
  // double-counted in each hole's independent base_link chain.
  const TrackedInstanceKey cup_holder_key{"cup_holder", 0};
  const auto cup_holder_it = rolling_windows_.find(cup_holder_key);
  const bool cup_holder_valid =
    cup_holder_it != rolling_windows_.end() && cup_holder_it->second.last_stable.valid;

  // Read LIVE every call (not cached in config_), specifically so it can
  // be tuned via `ros2 param set` with no node restart — see this
  // param's own doc comment on DepthPerceptionConfig for why, and its
  // "does not actually change reachability" caveat. get_parameter_or
  // since it's optional/absent from sim's own yaml (real-only) —
  // automatically_declare_parameters_from_overrides(true) means an
  // undeclared key would otherwise throw via get_parameter().
  double instance_tf_z_offset_m = 0.0;
  get_parameter_or("instance_tf_z_offset_m", instance_tf_z_offset_m, 0.0);

  if (cup_holder_valid) {
    const auto & cup_holder_point = cup_holder_it->second.last_stable;
    tf2::Transform camera_to_cup_holder(
      tf2::Quaternion::getIdentity(),
      tf2::Vector3(cup_holder_point.x, cup_holder_point.y, cup_holder_point.z));
    const tf2::Transform known_to_cup_holder = known_to_camera * camera_to_cup_holder;

    geometry_msgs::msg::TransformStamped cup_holder_tf;
    cup_holder_tf.header.stamp = header.stamp;
    cup_holder_tf.header.frame_id = config_.known_chain_frame;
    cup_holder_tf.child_frame_id = "cup_holder";
    cup_holder_tf.transform.translation.x = known_to_cup_holder.getOrigin().x();
    cup_holder_tf.transform.translation.y = known_to_cup_holder.getOrigin().y();
    // instance_tf_z_offset_m (2026-07-30, default 0.0, live-read above) —
    // does NOT reduce cup_holder's actual distance from known_chain_frame,
    // only the reported/broadcast Z. hole_1..hole_4 inherit this
    // automatically since they're parented to cup_holder, not
    // known_chain_frame, directly (see this function's own comment on
    // that re-anchoring) — no separate offset needed for them.
    cup_holder_tf.transform.translation.z =
      known_to_cup_holder.getOrigin().z() + instance_tf_z_offset_m;
    cup_holder_tf.transform.rotation.w = 1.0;
    instance_tf_broadcaster_.sendTransform(cup_holder_tf);
  }

  for (const auto & [key, window] : rolling_windows_) {
    if (key.class_name != "hole" || !window.last_stable.valid) {
      continue;
    }

    // No cup_holder to anchor to yet this cycle — skip holes entirely
    // rather than falling back to a base_link chain, so a hole TF is
    // never silently published in a different, inconsistent parent frame
    // from one cycle to the next.
    if (!cup_holder_valid) {
      continue;
    }

    const auto & cup_holder_point = cup_holder_it->second.last_stable;
    // Both points are camera-frame, identity-rotation positions (see
    // BackProjectedPoint's doc comment) — plain vector subtraction gives
    // the hole's offset from the holder's centroid, still expressed along
    // the camera frame's axes. Matrix3x3::operator* rotates a bare Vector3
    // (there is no Matrix3x3 * Transform overload — only Transform's own
    // operator* composes two Transforms) — exactly what's needed here
    // since camera_to_offset is translation-only anyway: rotate the offset
    // vector into known_chain_frame's axes via known_to_camera's rotation,
    // no translation component involved (a hole's offset FROM cup_holder
    // must not be shifted by cup_holder's own position again).
    const tf2::Vector3 camera_offset(
      window.last_stable.x - cup_holder_point.x,
      window.last_stable.y - cup_holder_point.y,
      window.last_stable.z - cup_holder_point.z);
    const tf2::Vector3 cup_holder_to_hole = known_to_camera.getBasis() * camera_offset;

    geometry_msgs::msg::TransformStamped hole_tf;
    hole_tf.header.stamp = header.stamp;
    hole_tf.header.frame_id = "cup_holder";
    hole_tf.child_frame_id = "hole_" + std::to_string(key.hole_number);
    hole_tf.transform.translation.x = cup_holder_to_hole.x();
    hole_tf.transform.translation.y = cup_holder_to_hole.y();
    hole_tf.transform.translation.z = cup_holder_to_hole.z();
    // Position-only — no orientation estimate exists for cup_holder/hole
    // (see BackProjectedPoint's own doc comment: position only, no
    // orientation, no averaging-across-frames beyond the rolling window),
    // so this TF's rotation is left as the identity quaternion rather than
    // fabricating a meaningless one.
    hole_tf.transform.rotation.w = 1.0;

    instance_tf_broadcaster_.sendTransform(hole_tf);
  }
}

void DepthPerceptionNode::reprojectToPixels(
  const BackProjectedPoint & point, double & out_px, double & out_py) const
{
  // Inverse of backProjectDetection()'s pinhole math: u = X * fx / Z +
  // cx_intrinsic, v = Y * fy / Z + cy_intrinsic. Guards against a
  // degenerate Z (shouldn't happen for a point that already passed
  // backProjectDetection's own validity checks, but avoids a division by
  // zero if ever called with a default-constructed BackProjectedPoint).
  if (std::abs(point.z) < 1e-6) {
    out_px = 0.0;
    out_py = 0.0;
    return;
  }
  out_px = (point.x * fx_ / point.z) + cx_intrinsic_;
  out_py = (point.y * fy_ / point.z) + cy_intrinsic_;
}

void DepthPerceptionNode::autoCalibrateStatusCallback(
  const visual_calibration_msgs::msg::AutoCalibrateStatus::ConstSharedPtr & msg)
{
  const bool was_paused = calibration_paused_;
  calibration_paused_ =
    (msg->phase == visual_calibration_msgs::msg::AutoCalibrateStatus::PHASE_RUNNING);

  if (calibration_paused_ && !was_paused) {
    RCLCPP_INFO(
      get_logger(),
      "~/auto_calibrate started (stage: '%s') — pausing detections_2d processing.",
      msg->stage.c_str());
  } else if (!calibration_paused_ && was_paused) {
    RCLCPP_INFO(
      get_logger(),
      "~/auto_calibrate finished (success: %s) — resuming detections_2d processing.",
      msg->success ? "true" : "false");
  }
}

BackProjectedPoint RollingWindow::median() const
{
  BackProjectedPoint result;
  if (samples.empty()) {
    return result;
  }

  // Independent per-axis median (see this struct's own doc comment for
  // why not a true multivariate median) — a fresh, sorted copy per axis
  // rather than sorting `samples` itself in place, since it's a deque of
  // full BackProjectedPoint structs, not bare doubles.
  std::vector<double> xs, ys, zs;
  xs.reserve(samples.size());
  ys.reserve(samples.size());
  zs.reserve(samples.size());
  for (const auto & sample : samples) {
    xs.push_back(sample.x);
    ys.push_back(sample.y);
    zs.push_back(sample.z);
  }
  std::sort(xs.begin(), xs.end());
  std::sort(ys.begin(), ys.end());
  std::sort(zs.begin(), zs.end());

  const size_t mid = samples.size() / 2;
  result.x = xs[mid];
  result.y = ys[mid];
  result.z = zs[mid];
  result.valid = true;
  return result;
}

double RollingWindow::distance(const BackProjectedPoint & a, const BackProjectedPoint & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool RollingWindow::updateLastStable(const BackProjectedPoint & new_median, double drift_threshold_m)
{
  if (!last_stable.valid) {
    // First sample ever seen for this instance — nothing to compare
    // against, so this new median simply becomes the initial known-good
    // position.
    last_stable = new_median;
    last_update_drifted = true;
    return true;
  }

  if (distance(new_median, last_stable) > drift_threshold_m) {
    // A genuine position change (e.g. the physical object moved) rather
    // than per-frame jitter around an already-known-good position.
    last_stable = new_median;
    last_update_drifted = true;
    return true;
  }

  // Close enough to the held position to be noise — last_stable is left
  // untouched. This is the core of the "hold last known position" fix:
  // small fluctuations never move the reported position at all.
  last_update_drifted = false;
  return false;
}

BackProjectedPoint DepthPerceptionNode::updateRollingWindow(
  const std::string & class_name, int32_t hole_number, const BackProjectedPoint & point,
  size_t & out_window_size, bool & out_drifted)
{
  RollingWindow & window = rolling_windows_[TrackedInstanceKey{class_name, hole_number}];
  window.push(point, static_cast<size_t>(config_.rolling_window_size));
  out_window_size = window.samples.size();
  out_drifted = window.updateLastStable(window.median(), config_.stable_drift_threshold_m);
  return window.last_stable;
}

BackProjectedPoint DepthPerceptionNode::backProjectDetection(double cx, double cy) const
{
  BackProjectedPoint result;

  std::lock_guard<std::mutex> lock(depth_mutex_);
  if (latest_depth_.empty()) {
    return result;
  }

  const int u = static_cast<int>(std::lround(cx));
  const int v = static_cast<int>(std::lround(cy));
  const int half = config_.depth_patch_half_size_px;

  const int u0 = std::max(0, u - half);
  const int v0 = std::max(0, v - half);
  const int u1 = std::min(latest_depth_.cols - 1, u + half);
  const int v1 = std::min(latest_depth_.rows - 1, v + half);

  if (u0 > u1 || v0 > v1) {
    // The detection's pixel center falls entirely outside the depth
    // image (e.g. a mismatched RGB/depth resolution) — nothing to sample.
    return result;
  }

  // Collect every finite, positive depth reading in the patch. Zero/NaN
  // values are Gazebo/RealSense's usual "no return at this pixel" signal
  // (common right at a cavity's rim, per Detection2D.msg's doc comment),
  // not a real distance of zero — they're excluded rather than dragging
  // the reduction toward zero.
  std::vector<float> samples;
  samples.reserve(static_cast<size_t>((u1 - u0 + 1) * (v1 - v0 + 1)));
  for (int v = v0; v <= v1; ++v) {
    for (int u = u0; u <= u1; ++u) {
      const float depth = latest_depth_.at<float>(v, u);
      if (std::isfinite(depth) && depth > 0.0f) {
        samples.push_back(depth);
      }
    }
  }

  if (samples.empty()) {
    return result;
  }

  // Median (not mean) — robust to a minority of outlier reads at a
  // cavity's rim, where some pixels in the patch may see the tray's flat
  // surface and others see the hole's true (farther) bottom.
  std::sort(samples.begin(), samples.end());
  const float depth_m = samples[samples.size() / 2];

  // Standard pinhole back-projection: a pixel (u, v) at known depth Z
  // corresponds to a 3D point (X, Y, Z) in the camera's optical frame
  // where X = (u - cx_intrinsic) * Z / fx, Y = (v - cy_intrinsic) * Z / fy.
  result.x = (cx - cx_intrinsic_) * depth_m / fx_;
  result.y = (cy - cy_intrinsic_) * depth_m / fy_;
  result.z = depth_m;
  result.valid = true;
  return result;
}

DepthPerceptionConfig DepthPerceptionNode::loadConfigFromParams() const
{
  DepthPerceptionConfig config;
  config.rgb_image_topic = get_parameter("rgb_image_topic").as_string();
  config.depth_image_topic = get_parameter("depth_image_topic").as_string();
  config.camera_info_topic = get_parameter("camera_info_topic").as_string();
  config.detections_2d_topic = get_parameter("detections_2d_topic").as_string();
  config.depth_scale_to_meters = get_parameter("depth_scale_to_meters").as_double();
  config.depth_patch_half_size_px =
    static_cast<int>(get_parameter("depth_patch_half_size_px").as_int());
  config.pause_while_calibration = get_parameter("pause_while_calibration").as_bool();
  if (config.pause_while_calibration) {
    config.auto_calibrate_status_topic =
      get_parameter("auto_calibrate_status_topic").as_string();
  }
  config.rolling_window_size = static_cast<int>(get_parameter("rolling_window_size").as_int());
  config.stable_drift_threshold_m = get_parameter("stable_drift_threshold_m").as_double();
  config.stable_positions_topic = get_parameter("stable_positions_topic").as_string();

  config.known_chain_frame = get_parameter("known_chain_frame").as_string();
  config.broadcast_frame_suffix = get_parameter("broadcast_frame_suffix").as_string();
  config.broadcast_instance_tfs = get_parameter("broadcast_instance_tfs").as_bool();

  return config;
}

}  // namespace depth_perception
