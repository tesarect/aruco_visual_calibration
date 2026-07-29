#ifndef ARUCO_PERCEPTION__CUP_HOLDER_DETECTOR_NODE_HPP_
#define ARUCO_PERCEPTION__CUP_HOLDER_DETECTOR_NODE_HPP_

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <visual_calibration_msgs/msg/detection2_d_array.hpp>

namespace aruco_perception
{

/// Tuning for CupHolderDetectorNode, loaded from a parameter file — sim
/// ONLY (this node has no real-robot equivalent; real keeps YOLO for
/// cup_holder/hole, see class doc comment). All thresholds are live
/// re-read per frame (never cached), so `ros2 param set` takes effect on
/// the very next frame with no restart — needed since sim lighting/
/// contrast is controlled but the exact HSV/gray cutoffs for the cup_holder
/// disc and its 4 holes still need empirical, iterative tuning against the
/// live camera feed rather than a one-shot guess.
struct CupHolderDetectorConfig
{
  std::string image_topic;
  /// Same topic aruco_detector_node/yolo_marker_bridge_node already
  /// publish Detection2DArray on — this node adds "cup_holder"/"hole"
  /// entries to the SAME stream (default "/aruco_perception/detections_2d").
  std::string detections_2d_topic;

  /// Optional BGR overlay image (drawn contours/labels), for visual
  /// verification via rqt_image_view while tuning thresholds live. Off by
  /// default — a debugging aid, not something downstream nodes consume.
  bool publish_overlay_image = false;
  std::string overlay_image_topic;

  /// Grayscale threshold [0,255]: pixels >= this are considered part of
  /// the light/white cup_holder disc (cv::threshold, THRESH_BINARY). The
  /// disc is the brightest large object in frame, so a single global
  /// threshold (not adaptive) is the starting point — see class doc
  /// comment for why adaptive thresholding (aruco_detector_node's
  /// approach) isn't reused here.
  int cup_holder_thresh = 180;

  /// Minimum circularity (4*pi*area/perimeter^2, 1.0 = perfect circle) for
  /// a contour to be accepted as the cup_holder disc. Filters out
  /// non-circular bright blobs (specular highlights, background clutter).
  double cup_holder_min_circularity = 0.7;
  /// Minimum contour area in pixels^2 — filters out small bright specks
  /// before circularity is even computed (cheap first-pass rejection).
  double cup_holder_min_area_px = 400.0;

  /// Grayscale threshold [0,255]: pixels <= this, WITHIN the cup_holder's
  /// own bounding region, are considered part of a hole (dark cavity vs.
  /// the disc's light surface). Separate from cup_holder_thresh since
  /// holes and disc are opposite ends of the brightness range, not the
  /// same cutoff inverted (the disc's own cast shadow / anti-aliased edge
  /// pixels sit in between — verify empirically against the live feed).
  int hole_thresh = 90;
  double hole_min_circularity = 0.6;
  /// Radius floor in pixels — separates the 4 real holes from small
  /// decorative screw-holes visible in the reference image (see class doc
  /// comment). Verify this empirically against live sim; not guessed
  /// blind.
  double hole_min_radius_px = 4.0;
  double hole_max_radius_px = 40.0;

  /// Startup default for the "active" parameter — true = this node runs
  /// detection on every frame. Live re-read every frame (see
  /// imageCallback), same pattern as aruco_detector_node's "active", so
  /// runtime set_parameters calls take effect with no restart. Unlike
  /// aruco_detector_node, nothing currently flips this — cup_holder/hole
  /// has no classical/hybrid switch (real never runs this node at all,
  /// see class doc comment) — kept only for a consistent
  /// pause-without-restart escape hatch during live tuning.
  bool active = true;
};

/// Vision-only node, SIMULATION ONLY: detects the cup_holder disc (1
/// white/light circular object) and its up to 4 holes via classical
/// OpenCV (threshold + contour + circularity), publishing
/// visual_calibration_msgs/Detection2DArray on the exact same topic
/// aruco_detector_node/yolo_marker_bridge_node already publish on. 2D
/// pixel space only — no depth, no TF, no 3D pose (see
/// depth_perception_node for the consumer that does that).
///
/// Why this node exists, and why it is sim-only: real's cup_holder/hole
/// detection runs on YOLO (aruco_perception_yolo_bridge's
/// yolo_marker_bridge_node, backed by YOLO-pipeline/inference_server.py).
/// Sim's rosject has no GPU and is already CPU-oversubscribed running
/// Gazebo+RViz+the web dashboard simultaneously, making YOLO inference
/// there too slow for continuous use. This node is a drop-in ALTERNATE
/// PUBLISHER on the same interface for sim only — depth_perception_node
/// (the actual consumer) has zero awareness of which detector produced a
/// given Detection2DArray message. This node is never launched on real
/// (no cup_holder_detector_real.yaml exists, and no real_tmux_*.sh script
/// references it) — that absence, not any runtime gate, is what keeps it
/// off the real robot.
///
/// Adaptive thresholding (aruco_detector_node's approach, tuned for
/// uneven real-world lighting) is deliberately NOT used here: sim
/// lighting is controlled/consistent, and the cup_holder disc's
/// brightness relative to its background is the whole signal this node
/// depends on — a single tunable global threshold is simpler to reason
/// about and tune live via `ros2 param set`, and is the right complexity
/// level for a controlled sim environment specifically.
///
/// Hole quadrant numbering: ported from yolo_marker_bridge_node.py's
/// assign_hole_quadrants() (1=top-left, 2=top-right, 3=bottom-left,
/// 4=bottom-right, split around the cup_holder's own bbox center). NOTE:
/// that function's design comment justifies a simple 2-axis split because
/// real's camera is wall-fixed and never rolls — sim's wrist-mounted
/// camera can roll through a wider range of orientations during a scan,
/// so this numbering has NOT yet been empirically verified to stay
/// consistent across every pose sim's arm actually reaches. Treat
/// hole_number as provisional until checked live against a real scan
/// sweep.
class CupHolderDetectorNode : public rclcpp::Node
{
public:
  CupHolderDetectorNode();

private:
  CupHolderDetectorConfig loadConfigFromParams() const;

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  /// One accepted circular contour, in pixel space — the common shape
  /// both the cup_holder pass and the hole pass reduce a cv::findContours
  /// result down to before building Detection2D messages.
  struct CircleCandidate
  {
    double cx;
    double cy;
    double radius;
    double circularity;
    std::vector<cv::Point> contour;
  };

  /// Finds all contours in `binary` passing the given area/circularity
  /// filters, sorted by descending area. `binary` must already be a
  /// single-channel thresholded (0/255) image.
  static std::vector<CircleCandidate> findCircularContours(
    const cv::Mat & binary, double min_area_px, double min_circularity);

  /// Ported from yolo_marker_bridge_node.py's assign_hole_quadrants() —
  /// see class doc comment for the sim-specific caveat on this algorithm.
  /// Mutates each element of `holes` in place, setting hole_number.
  static void assignHoleQuadrants(
    std::vector<visual_calibration_msgs::msg::Detection2D> & holes,
    bool cup_holder_found, double cup_holder_cx, double cup_holder_cy);

  CupHolderDetectorConfig config_;

  image_transport::Subscriber image_sub_;
  rclcpp::Publisher<visual_calibration_msgs::msg::Detection2DArray>::SharedPtr detections_2d_pub_;
  image_transport::Publisher overlay_image_pub_;
};

}  // namespace aruco_perception

#endif  // ARUCO_PERCEPTION__CUP_HOLDER_DETECTOR_NODE_HPP_
