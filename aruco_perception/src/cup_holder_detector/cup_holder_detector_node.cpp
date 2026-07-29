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

void CupHolderDetectorNode::assignHoleQuadrants(
  std::vector<visual_calibration_msgs::msg::Detection2D> & holes,
  bool cup_holder_found, double cup_holder_cx, double cup_holder_cy)
{
  if (holes.empty()) {
    return;
  }

  double ref_x = cup_holder_cx;
  double ref_y = cup_holder_cy;
  if (!cup_holder_found) {
    // Fall back to the mean (cx, cy) of this frame's own hole detections —
    // ported exactly from yolo_marker_bridge_node.py's
    // assign_hole_quadrants(), see that function's doc comment.
    double sum_x = 0.0, sum_y = 0.0;
    for (const auto & hole : holes) {
      sum_x += hole.cx;
      sum_y += hole.cy;
    }
    ref_x = sum_x / static_cast<double>(holes.size());
    ref_y = sum_y / static_cast<double>(holes.size());
  }

  for (auto & hole : holes) {
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
  const int cup_holder_thresh = static_cast<int>(get_parameter("cup_holder_thresh").as_int());
  const double cup_holder_min_circularity =
    get_parameter("cup_holder_min_circularity").as_double();
  const double cup_holder_min_area_px = get_parameter("cup_holder_min_area_px").as_double();
  const int hole_thresh = static_cast<int>(get_parameter("hole_thresh").as_int());
  const double hole_min_circularity = get_parameter("hole_min_circularity").as_double();
  const double hole_min_radius_px = get_parameter("hole_min_radius_px").as_double();
  const double hole_max_radius_px = get_parameter("hole_max_radius_px").as_double();

  // Pass 1 — cup_holder: the disc is the brightest large circular object
  // in frame. THRESH_BINARY: pixels >= cup_holder_thresh -> 255.
  cv::Mat cup_holder_binary;
  cv::threshold(gray, cup_holder_binary, cup_holder_thresh, 255, cv::THRESH_BINARY);
  const std::vector<CircleCandidate> cup_holder_candidates =
    findCircularContours(cup_holder_binary, cup_holder_min_area_px, cup_holder_min_circularity);

  const bool cup_holder_found = !cup_holder_candidates.empty();
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

  assignHoleQuadrants(
    holes, cup_holder_found,
    cup_holder_found ? cup_holder->cx : 0.0, cup_holder_found ? cup_holder->cy : 0.0);
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
      cv::putText(
        overlay_ptr->image, std::to_string(hole.hole_number),
        cv::Point2d(hole.cx - 5, hole.cy + 5), cv::FONT_HERSHEY_SIMPLEX, 0.6,
        cv::Scalar(255, 255, 255), 2);
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

  config.cup_holder_thresh = static_cast<int>(get_parameter("cup_holder_thresh").as_int());
  config.cup_holder_min_circularity = get_parameter("cup_holder_min_circularity").as_double();
  config.cup_holder_min_area_px = get_parameter("cup_holder_min_area_px").as_double();

  config.hole_thresh = static_cast<int>(get_parameter("hole_thresh").as_int());
  config.hole_min_circularity = get_parameter("hole_min_circularity").as_double();
  config.hole_min_radius_px = get_parameter("hole_min_radius_px").as_double();
  config.hole_max_radius_px = get_parameter("hole_max_radius_px").as_double();

  config.active = get_parameter("active").as_bool();
  return config;
}

}  // namespace aruco_perception
