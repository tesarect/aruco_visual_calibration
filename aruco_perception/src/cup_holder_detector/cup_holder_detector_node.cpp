#include "aruco_perception/cup_holder_detector_node.hpp"

#include <algorithm>
#include <cmath>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>

namespace aruco_perception
{

CupHolderDetectorNode::CupHolderDetectorNode()
: Node(
    "cup_holder_detector_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
  config_(loadConfigFromParams())
{
  image_sub_ = image_transport::create_subscription(
    this, config_.image_topic,
    std::bind(&CupHolderDetectorNode::imageCallback, this, std::placeholders::_1),
    "raw");

  detections_2d_pub_ = create_publisher<visual_calibration_msgs::msg::Detection2DArray>(
    config_.detections_2d_topic, 10);

  if (config_.publish_overlay_image) {
    overlay_image_pub_ = image_transport::create_publisher(this, config_.overlay_image_topic);
    marker_overlay_sub_ = image_transport::create_subscription(
      this, config_.overlay_image_input_topic,
      std::bind(&CupHolderDetectorNode::markerOverlayCallback, this, std::placeholders::_1),
      "raw");
  }

  RCLCPP_INFO(get_logger(), "cup_holder_detector_node ready (sim-only classical CV detector).");
}

namespace
{
/// 4*pi*area/perimeter^2 — 1.0 for a perfect circle, lower for irregular/
/// elongated shapes. perimeter == 0 (degenerate contour) returns 0.0
/// rather than dividing by zero.
double circularity(double area, double perimeter)
{
  if (perimeter <= 0.0) {
    return 0.0;
  }
  return 4.0 * CV_PI * area / (perimeter * perimeter);
}
}  // namespace

std::vector<CupHolderDetectorNode::CircleCandidate> CupHolderDetectorNode::findCircularContours(
  const cv::Mat & binary, double min_area_px, double min_circularity)
{
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

  std::vector<CircleCandidate> candidates;
  for (const auto & contour : contours) {
    const double area = cv::contourArea(contour);
    if (area < min_area_px) {
      continue;
    }
    const double perimeter = cv::arcLength(contour, true);
    const double circ = circularity(area, perimeter);
    if (circ < min_circularity) {
      continue;
    }

    cv::Point2f center;
    float radius = 0.0f;
    cv::minEnclosingCircle(contour, center, radius);

    candidates.push_back(
      CircleCandidate{
        static_cast<double>(center.x), static_cast<double>(center.y),
        static_cast<double>(radius), circ, contour});
  }

  std::sort(
    candidates.begin(), candidates.end(),
    [](const CircleCandidate & a, const CircleCandidate & b) {
      return (a.radius * a.radius) > (b.radius * b.radius);
    });
  return candidates;
}

void CupHolderDetectorNode::refineCupHolderCircle(CircleCandidate & candidate)
{
  // fitEllipse requires >= 5 points — every contour reaching this point
  // already passed cup_holder_min_area_px (default 400px^2), which in
  // practice always yields far more than 5 boundary points, but guard
  // anyway rather than let OpenCV throw.
  if (candidate.contour.size() < 5) {
    return;
  }
  const cv::RotatedRect ellipse = cv::fitEllipse(candidate.contour);
  candidate.cx = ellipse.center.x;
  candidate.cy = ellipse.center.y;
  // size.width/height are the full major/minor axis lengths — average the
  // two SEMI-axes (half of each) into one representative radius. See
  // this function's declaration comment for why: minEnclosingCircle
  // stretches to cover a rim/cylinder-edge outlier bulge in the contour,
  // fitEllipse's least-squares fit does not.
  candidate.radius = (ellipse.size.width + ellipse.size.height) / 4.0;
}

void CupHolderDetectorNode::assignHoleQuadrants(
  std::vector<visual_calibration_msgs::msg::Detection2D> & holes)
{
  if (holes.empty()) {
    // Nothing to label this frame — also nothing worth remembering for
    // next frame's identity match (an empty previous_holes_ next frame
    // just means every hole falls back to the from-scratch quadrant
    // split below, same as today's first-ever frame).
    previous_holes_.clear();
    return;
  }

  // --- Label-flicker fix (2026-07-30) -------------------------------
  // BUG (confirmed live + by code trace): the quadrant split below is a
  // pure function of THIS frame's positions relative to THIS frame's own
  // reference point, recomputed from scratch every frame with no memory
  // of which physical hole previously held which number. A hole sitting
  // near the ref_x or ref_y midline can have its top/left boolean flip
  // from frame to frame on nothing more than a few pixels of detection
  // jitter (contour noise, a slightly different Canny/threshold outcome,
  // etc.) even though the physical hole hasn't meaningfully moved.
  // depth_perception_node keys its rolling_windows_ map by
  // TrackedInstanceKey{class_name, hole_number} — if hole_number flips,
  // TWO DIFFERENT physical holes end up writing into the SAME rolling
  // window across different frames, corrupting that one window's tracked
  // position with data from two different objects, while holes far from
  // any boundary (confidently classified every frame) stay stable. This
  // matches the reported symptom exactly: hole_1/2/4 stable, hole_3
  // (or whichever number sits at the boundary for this arrangement)
  // intermittent/wrong.
  //
  // FIX: give each physical hole a persistent identity across frames
  // instead of reclassifying from scratch. Match this frame's holes to
  // previous_holes_ (last frame's output, already labeled) by nearest-
  // centroid-distance, and inherit the matched previous hole's
  // hole_number directly — no top/left recomputation at all for a
  // matched hole, so jitter around the ref_x/ref_y midline can no longer
  // flip its label (the label isn't even a function of the midline once
  // a hole has an established identity). Only holes with NO confident
  // previous-frame match (first sighting of a hole, or previous_holes_ is
  // empty because last frame had 0 holes) fall back to the original
  // quadrant-split logic to bootstrap an initial label.
  //
  // Why nearest-centroid matching (not just "add a deadband around the
  // midline"): a deadband only prevents a flip for a hole oscillating
  // right at the boundary within one frame's jitter — it does nothing
  // for the case of a hole legitimately, slowly drifting across the
  // midline over many frames (e.g. during a slow wrist-camera scan
  // sweep), where jitter can still occur right at the crossing point.
  // Persistent identity via nearest-centroid handles both cases
  // uniformly: a hole's label only changes when it's unambiguously
  // closer to a DIFFERENT previous hole than to its own previous
  // position, which requires far more motion than one frame of jitter.
  //
  // Match gating (hole_reassign_max_dist_px, default = hole_max_radius_px,
  // see its own doc comment for the scale justification): only accept a
  // previous-frame match within this pixel distance. A greedy
  // nearest-available match (holes processed in ascending distance order,
  // each previous hole usable at most once) — 4 holes max, so an O(n^2)
  // greedy pass is more than fast enough and there's no need for a full
  // Hungarian assignment here.
  const double max_dist_px = get_parameter("hole_reassign_max_dist_px").as_double();
  const double max_dist_sq = max_dist_px * max_dist_px;

  std::vector<bool> prev_used(previous_holes_.size(), false);
  std::vector<bool> matched(holes.size(), false);

  // Build all (candidate hole index, candidate previous index, dist_sq)
  // triples within the gate, then greedily consume them in ascending
  // distance order — this is what keeps the match a true nearest-
  // neighbor pairing even when several holes are within the gate of each
  // other (e.g. two holes both close to a third's previous position):
  // the single closest pair overall is matched first and removed from
  // further consideration, then the next-closest remaining pair, etc.
  struct Candidate
  {
    double dist_sq;
    size_t hole_idx;
    size_t prev_idx;
  };
  std::vector<Candidate> candidates;
  for (size_t i = 0; i < holes.size(); ++i) {
    for (size_t j = 0; j < previous_holes_.size(); ++j) {
      const double dx = holes[i].cx - previous_holes_[j].cx;
      const double dy = holes[i].cy - previous_holes_[j].cy;
      const double dist_sq = dx * dx + dy * dy;
      if (dist_sq <= max_dist_sq) {
        candidates.push_back(Candidate{dist_sq, i, j});
      }
    }
  }
  std::sort(
    candidates.begin(), candidates.end(),
    [](const Candidate & a, const Candidate & b) { return a.dist_sq < b.dist_sq; });

  for (const auto & c : candidates) {
    if (matched[c.hole_idx] || prev_used[c.prev_idx]) {
      continue;
    }
    holes[c.hole_idx].hole_number = previous_holes_[c.prev_idx].hole_number;
    matched[c.hole_idx] = true;
    prev_used[c.prev_idx] = true;
  }

  // Bootstrap path — original from-scratch quadrant split, applied ONLY
  // to holes that found no acceptable previous-frame match above (new
  // hole appearing for the first time, or previous_holes_ was empty).
  // Reference point: ALWAYS the mean (cx, cy) of THIS frame's own hole
  // detections — NOT the cup_holder's own fitted center. CHANGED
  // 2026-07-30: live-tested that the disc's rim contour is genuinely
  // elliptical from sim's wrist-camera angle (perspective foreshortening —
  // confirmed axes ~220x226px, not an artifact of dilate kernel size,
  // consistent across 2/3/5px dilate), which pulls a fitEllipse/moments-
  // based disc center away from the true visual center of the 4-hole
  // arrangement. The 4 holes themselves detect cleanly and accurately via
  // simple thresholding (no equivalent distortion) — using their own
  // centroid as the quadrant reference sidesteps the disc-fit distortion
  // entirely, at the cost of needing at least 1 hole detected (never an
  // issue in practice: quadrant assignment is meaningless with 0 holes
  // anyway, see the empty check above). This diverges from
  // yolo_marker_bridge_node.py's assign_hole_quadrants(), which prefers
  // the cup_holder's bbox center and only falls back to the hole-centroid
  // when no cup_holder was found that frame — real's camera doesn't
  // appear to hit this same perspective-distortion problem (wall-mounted,
  // different geometry), so that node's own preference order is left
  // unchanged; this is a SIM-specific divergence, not a claim that real's
  // logic needs the same fix.
  double sum_x = 0.0, sum_y = 0.0;
  for (const auto & hole : holes) {
    sum_x += hole.cx;
    sum_y += hole.cy;
  }
  const double ref_x = sum_x / static_cast<double>(holes.size());
  const double ref_y = sum_y / static_cast<double>(holes.size());

  for (size_t i = 0; i < holes.size(); ++i) {
    if (matched[i]) {
      continue;
    }
    auto & hole = holes[i];
    const bool top = hole.cy < ref_y;
    const bool left = hole.cx < ref_x;
    if (top && left) {
      hole.hole_number = 1;
    } else if (top && !left) {
      hole.hole_number = 2;
    } else if (!top && left) {
      hole.hole_number = 3;
    } else {
      hole.hole_number = 4;
    }
  }

  // Remember this frame's labeled output as next frame's identity
  // reference — see previous_holes_'s own doc comment.
  previous_holes_ = holes;
}

void CupHolderDetectorNode::markerOverlayCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  // Cache only — see class doc comment. Conversion failures here must not
  // crash detection; just skip this frame's cache update and keep whatever
  // was cached before (or nullptr, falling back to the raw-frame path in
  // imageCallback).
  try {
    latest_marker_overlay_ = cv_bridge::toCvCopy(msg, "bgr8");
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "cv_bridge conversion failed for marker overlay input: %s", e.what());
  }
}

void CupHolderDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  // Live re-read (never cached in config_) so `ros2 param set` takes
  // effect on the very next frame — same pattern as aruco_detector_node's
  // "active" (see class doc comment).
  if (!get_parameter("active").as_bool()) {
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "cv_bridge conversion failed: %s", e.what());
    return;
  }
  const cv::Mat & gray = cv_ptr->image;

  // Live re-read every tunable, every frame — see CupHolderDetectorConfig's
  // doc comment for why (iterative live tuning via `ros2 param set`, no
  // restart).
  const int cup_holder_canny_low = static_cast<int>(get_parameter("cup_holder_canny_low").as_int());
  const int cup_holder_canny_high =
    static_cast<int>(get_parameter("cup_holder_canny_high").as_int());
  const int cup_holder_blur_kernel_px =
    static_cast<int>(get_parameter("cup_holder_blur_kernel_px").as_int());
  const int cup_holder_dilate_kernel_px =
    static_cast<int>(get_parameter("cup_holder_dilate_kernel_px").as_int());
  const double cup_holder_min_circularity =
    get_parameter("cup_holder_min_circularity").as_double();
  const double cup_holder_min_area_px = get_parameter("cup_holder_min_area_px").as_double();
  const int hole_thresh = static_cast<int>(get_parameter("hole_thresh").as_int());
  const double hole_min_circularity = get_parameter("hole_min_circularity").as_double();
  const double hole_min_radius_px = get_parameter("hole_min_radius_px").as_double();
  const double hole_max_radius_px = get_parameter("hole_max_radius_px").as_double();

  // Pass 1 — cup_holder: found via its RIM (edge), not flat brightness.
  // 2026-07-30: live-tested that the disc has almost no brightness
  // separation from the background wall in sim's actual lighting — no
  // single cv::threshold cutoff can isolate it — but Canny cleanly finds
  // its rim as a closed loop (confirmed via cup_holder_pipeline_debug.py).
  // Blur first to avoid fragmenting the edge on per-pixel noise, dilate
  // after to bridge any small gaps in the 1px Canny line so
  // findContours sees one continuous closed shape.
  cv::Mat cup_holder_blurred;
  const cv::Size blur_kernel(cup_holder_blur_kernel_px, cup_holder_blur_kernel_px);
  cv::GaussianBlur(gray, cup_holder_blurred, blur_kernel, 0);

  cv::Mat cup_holder_edges;
  cv::Canny(cup_holder_blurred, cup_holder_edges, cup_holder_canny_low, cup_holder_canny_high);

  cv::Mat cup_holder_edges_dilated;
  const cv::Mat dilate_kernel = cv::Mat::ones(
    cup_holder_dilate_kernel_px, cup_holder_dilate_kernel_px, CV_8U);
  cv::dilate(cup_holder_edges, cup_holder_edges_dilated, dilate_kernel);

  std::vector<CircleCandidate> cup_holder_candidates = findCircularContours(
    cup_holder_edges_dilated, cup_holder_min_area_px, cup_holder_min_circularity);

  // Canny+dilate on a rim produces TWO concentric contours per real edge
  // (the inner and outer boundary of the thickened line) — both pass the
  // same area/circularity filters, so findCircularContours' largest-first
  // sort naturally picks the outer one as front() without extra dedup
  // logic; the inner duplicate is simply never used.
  const bool cup_holder_found = !cup_holder_candidates.empty();
  if (cup_holder_found) {
    // Re-fit the winning contour with fitEllipse — see
    // refineCupHolderCircle's doc comment for why minEnclosingCircle
    // overshoots the disc's true flat-top boundary.
    refineCupHolderCircle(cup_holder_candidates.front());
  }
  const CircleCandidate * cup_holder =
    cup_holder_found ? &cup_holder_candidates.front() : nullptr;

  // Pass 2 — holes: dark cavities, searched only within a square ROI
  // around the cup_holder (if found) so background clutter outside the
  // disc can never be mistaken for a hole. Falls back to the full image
  // if no cup_holder was found this frame (still worth trying — holes may
  // be visible even on a frame where the disc's own threshold pass
  // missed).
  cv::Rect hole_roi(0, 0, gray.cols, gray.rows);
  if (cup_holder_found) {
    const int half = static_cast<int>(std::ceil(cup_holder->radius));
    const int x0 = std::max(0, static_cast<int>(cup_holder->cx) - half);
    const int y0 = std::max(0, static_cast<int>(cup_holder->cy) - half);
    const int x1 = std::min(gray.cols, static_cast<int>(cup_holder->cx) + half);
    const int y1 = std::min(gray.rows, static_cast<int>(cup_holder->cy) + half);
    hole_roi = cv::Rect(x0, y0, std::max(1, x1 - x0), std::max(1, y1 - y0));
  }

  cv::Mat hole_binary;
  cv::threshold(gray(hole_roi), hole_binary, hole_thresh, 255, cv::THRESH_BINARY_INV);
  std::vector<CircleCandidate> hole_candidates =
    findCircularContours(hole_binary, CV_PI * hole_min_radius_px * hole_min_radius_px,
      hole_min_circularity);

  // Build Detection2D messages. detections_2d is published EVERY frame,
  // empty detections[] when nothing is found — same continuous-stream
  // convention aruco_detector_node/yolo_marker_bridge_node already
  // established (see Detection2DArray.msg's header comment).
  visual_calibration_msgs::msg::Detection2DArray detections_msg;
  detections_msg.header = msg->header;

  if (cup_holder_found) {
    visual_calibration_msgs::msg::Detection2D detection;
    detection.class_name = "cup_holder";
    detection.cx = cup_holder->cx;
    detection.cy = cup_holder->cy;
    detection.confidence = cup_holder->circularity;
    detection.bbox = {
      cup_holder->cx - cup_holder->radius, cup_holder->cy - cup_holder->radius,
      cup_holder->cx + cup_holder->radius, cup_holder->cy + cup_holder->radius};
    detection.hole_number = 0;
    detections_msg.detections.push_back(detection);
  }

  std::vector<visual_calibration_msgs::msg::Detection2D> holes;
  for (const auto & candidate : hole_candidates) {
    if (candidate.radius < hole_min_radius_px || candidate.radius > hole_max_radius_px) {
      continue;
    }
    // Candidate coordinates are ROI-local — offset back to full-image
    // pixel space before publishing.
    const double cx = candidate.cx + hole_roi.x;
    const double cy = candidate.cy + hole_roi.y;

    visual_calibration_msgs::msg::Detection2D detection;
    detection.class_name = "hole";
    detection.cx = cx;
    detection.cy = cy;
    detection.confidence = candidate.circularity;
    detection.bbox = {
      cx - candidate.radius, cy - candidate.radius,
      cx + candidate.radius, cy + candidate.radius};
    detection.hole_number = 0;  // set below by assignHoleQuadrants
    holes.push_back(detection);
  }

  assignHoleQuadrants(holes);
  for (auto & hole : holes) {
    detections_msg.detections.push_back(hole);
  }

  detections_2d_pub_->publish(detections_msg);

  if (config_.publish_overlay_image) {
    cv_bridge::CvImagePtr overlay_ptr;
    // Draw on top of aruco_detector_node's latest cached marker-only
    // overlay when available (see class doc comment) — falls back to this
    // node's own raw frame if aruco_detector_node isn't running/hasn't
    // published yet, so the combined overlay still comes out (marker-less,
    // but not silently missing).
    if (latest_marker_overlay_) {
      overlay_ptr = std::make_shared<cv_bridge::CvImage>(
        latest_marker_overlay_->header, latest_marker_overlay_->encoding,
        latest_marker_overlay_->image.clone());
    } else {
      try {
        overlay_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      } catch (const cv_bridge::Exception & e) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000, "cv_bridge overlay conversion failed: %s", e.what());
        return;
      }
    }

    // Live re-read every frame (never cached) — see
    // CupHolderDetectorConfig::show_extras_markers's doc comment.
    const bool show_extras_markers = get_parameter("show_extras_markers").as_bool();

    // Base layer — ALWAYS drawn: filled centroid dot + readable label for
    // the cup_holder and each hole. Matches yolo_marker_bridge_node.py's
    // own always-on hole marker style exactly (black outline + green fill
    // text, filled green dot) — see class doc comment. real's overlay
    // never drew a cup_holder-specific marker at all; this adds one using
    // the same visual style for consistency.
    if (cup_holder_found) {
      const cv::Point2i center(
        static_cast<int>(cup_holder->cx), static_cast<int>(cup_holder->cy));
      cv::circle(overlay_ptr->image, center, 4, cv::Scalar(0, 255, 0), -1);
      const cv::Point2i text_pos(center.x + 8, center.y - 8);
      cv::putText(
        overlay_ptr->image, "cup_holder", text_pos, cv::FONT_HERSHEY_SIMPLEX, 0.6,
        cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
      cv::putText(
        overlay_ptr->image, "cup_holder", text_pos, cv::FONT_HERSHEY_SIMPLEX, 0.6,
        cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    }
    for (const auto & hole : holes) {
      const cv::Point2i center(static_cast<int>(hole.cx), static_cast<int>(hole.cy));
      cv::circle(overlay_ptr->image, center, 4, cv::Scalar(0, 255, 0), -1);
      const std::string label = std::to_string(hole.hole_number);
      const cv::Point2i text_pos(center.x + 8, center.y - 8);
      cv::putText(
        overlay_ptr->image, label, text_pos, cv::FONT_HERSHEY_SIMPLEX, 0.9,
        cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
      cv::putText(
        overlay_ptr->image, label, text_pos, cv::FONT_HERSHEY_SIMPLEX, 0.9,
        cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    }

    // Extras layer — gated behind show_extras_markers (default OFF): the
    // diagnostic detection-radius ring, distinct color (yellow/red) from
    // the base layer's green so both remain visually distinguishable when
    // both are on.
    if (show_extras_markers) {
      if (cup_holder_found) {
        cv::circle(
          overlay_ptr->image, cv::Point2d(cup_holder->cx, cup_holder->cy),
          static_cast<int>(cup_holder->radius), cv::Scalar(0, 255, 255), 2);
      }
      for (const auto & hole : holes) {
        const double radius = (hole.bbox[2] - hole.bbox[0]) / 2.0;
        cv::circle(
          overlay_ptr->image, cv::Point2d(hole.cx, hole.cy),
          static_cast<int>(radius), cv::Scalar(0, 0, 255), 2);
      }
    }

    overlay_image_pub_.publish(overlay_ptr->toImageMsg());
  }
}

CupHolderDetectorConfig CupHolderDetectorNode::loadConfigFromParams() const
{
  CupHolderDetectorConfig config;
  config.image_topic = get_parameter("image_topic").as_string();
  config.detections_2d_topic = get_parameter("detections_2d_topic").as_string();
  config.publish_overlay_image = get_parameter("publish_overlay_image").as_bool();
  config.overlay_image_input_topic = get_parameter("overlay_image_input_topic").as_string();
  config.overlay_image_topic = get_parameter("overlay_image_topic").as_string();
  config.show_extras_markers = get_parameter("show_extras_markers").as_bool();

  config.cup_holder_canny_low = static_cast<int>(get_parameter("cup_holder_canny_low").as_int());
  config.cup_holder_canny_high = static_cast<int>(get_parameter("cup_holder_canny_high").as_int());
  config.cup_holder_blur_kernel_px =
    static_cast<int>(get_parameter("cup_holder_blur_kernel_px").as_int());
  config.cup_holder_dilate_kernel_px =
    static_cast<int>(get_parameter("cup_holder_dilate_kernel_px").as_int());
  config.cup_holder_min_circularity = get_parameter("cup_holder_min_circularity").as_double();
  config.cup_holder_min_area_px = get_parameter("cup_holder_min_area_px").as_double();

  config.hole_thresh = static_cast<int>(get_parameter("hole_thresh").as_int());
  config.hole_min_circularity = get_parameter("hole_min_circularity").as_double();
  config.hole_min_radius_px = get_parameter("hole_min_radius_px").as_double();
  config.hole_max_radius_px = get_parameter("hole_max_radius_px").as_double();
  config.hole_reassign_max_dist_px = get_parameter("hole_reassign_max_dist_px").as_double();

  config.active = get_parameter("active").as_bool();
  return config;
}

}  // namespace aruco_perception
