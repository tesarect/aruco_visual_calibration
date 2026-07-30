#ifndef ARUCO_PERCEPTION__CUP_HOLDER_DETECTOR_NODE_HPP_
#define ARUCO_PERCEPTION__CUP_HOLDER_DETECTOR_NODE_HPP_

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
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
  /// Input: aruco_detector_node's marker-only overlay (sim's
  /// aruco_detector_sim.yaml reroutes its overlay_image_topic here instead
  /// of publishing /aruco_perception/overlay_image directly — see that
  /// file's comment). This node draws cup_holder/hole on top of whatever
  /// arrives here and republishes the combined image — see class doc
  /// comment for why (matching real's "one node, one combined image, one
  /// publish" pattern).
  std::string overlay_image_input_topic;
  /// Output: the actual, web-app-facing /aruco_perception/overlay_image —
  /// this node is the SOLE publisher of it on sim (see class doc comment).
  std::string overlay_image_topic;

  /// cv::Canny low/high threshold pair, run on a blurred grayscale image
  /// (see cup_holder_blur_kernel_px) to find the cup_holder disc's rim.
  /// REPLACES an earlier flat cv::threshold(THRESH_BINARY) approach
  /// (2026-07-30) — live-tested and confirmed via
  /// cup_holder_pipeline_debug.py that the disc has almost no brightness
  /// separation from the background wall in sim's actual lighting (the
  /// wall and the white disc surface are nearly the same grayscale value),
  /// so no single global brightness cutoff can isolate it — Canny finds
  /// the disc via its edge/rim instead, which IS distinct regardless of
  /// the flat-region brightness similarity. OpenCV's own docs recommend a
  /// high:low ratio of roughly 2:1 to 3:1.
  int cup_holder_canny_low = 80;
  int cup_holder_canny_high = 200;
  /// Square Gaussian blur kernel size (must be odd) applied before Canny —
  /// reduces per-pixel noise that would otherwise fragment the edge map
  /// into small broken segments.
  int cup_holder_blur_kernel_px = 5;
  /// Square dilation kernel size (pixels) applied to the raw Canny edge
  /// map before findContours — Canny edges are 1px wide and often have
  /// small gaps, so a rim that's a closed loop in reality won't close
  /// into one clean contour without this. Confirmed via
  /// cup_holder_pipeline_debug.py that a small dilate is sufficient (the
  /// rim closes cleanly at kernel=3) — keep this small; too large starts
  /// merging the disc's rim contour with nearby hole rims.
  int cup_holder_dilate_kernel_px = 3;

  /// Minimum circularity (4*pi*area/perimeter^2, 1.0 = perfect circle) for
  /// a contour to be accepted as the cup_holder disc. Filters out
  /// non-circular edges — confirmed live (2026-07-30) that this alone
  /// cleanly rejects a long diagonal background/wall-corner edge line
  /// (circularity ~0.01-0.11) while accepting the disc's own rim
  /// (circularity ~0.78-0.87 in the same test frame).
  double cup_holder_min_circularity = 0.6;
  /// Minimum contour area in pixels^2 — filters out small edge fragments
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
/// OpenCV, publishing visual_calibration_msgs/Detection2DArray on the
/// exact same topic aruco_detector_node/yolo_marker_bridge_node already
/// publish on. 2D pixel space only — no depth, no TF, no 3D pose (see
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
/// Detection approach differs between the disc and the holes, by design:
/// the 4 holes are reliably darker than everything else in frame, so a
/// flat grayscale threshold (cv::threshold, THRESH_BINARY_INV) isolates
/// them cleanly (see hole_thresh). The cup_holder DISC, however, was
/// live-tested (2026-07-30) to have almost no brightness separation from
/// the background wall in sim's actual lighting — no single global
/// cv::threshold cutoff could isolate it at any value tried — so the disc
/// is instead found via its RIM: cv::Canny edge detection (on a blurred
/// grayscale image) + a small dilate to close small gaps in the 1px edge
/// line + findContours/circularity, same as the holes' final
/// contour/circularity step but fed an edge map instead of a brightness
/// mask. Confirmed via a standalone debug tool
/// (resources/scripts/python/cup_holder_pipeline_debug.py) against a live
/// sim frame that this cleanly isolates the disc's rim as a closed loop
/// (circularity ~0.78-0.87) while correctly rejecting a nearby diagonal
/// wall-corner edge (circularity ~0.01-0.11) via the same circularity
/// filter already used for holes.
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
///
/// Overlay unification (2026-07-29): sim's aruco_detector_node no longer
/// publishes /aruco_perception/overlay_image directly — its
/// aruco_detector_sim.yaml reroutes overlay_image_topic to
/// overlay_image_input_topic here instead (see that file's comment). This
/// node subscribes to that marker-only overlay, caches the LATEST received
/// frame (latest_marker_overlay_), and — every time its OWN detection pass
/// completes — draws cup_holder/hole circles on top of that cached frame
/// and publishes the combined image as the sole publisher of
/// /aruco_perception/overlay_image. No strict frame-pairing/sync between
/// the two subscriptions: same "latest cached frame, degrades gracefully
/// if the other publisher is down" pattern yolo_marker_bridge_node.py
/// already uses for overlaying depth_perception_node's stable-position
/// markers (see that node's own doc comment) — if aruco_detector_node
/// isn't running/hasn't published yet, latest_marker_overlay_ is empty and
/// this node falls back to drawing directly on its own raw camera frame
/// (still a valid, if marker-less, overlay) rather than publishing
/// nothing.
class CupHolderDetectorNode : public rclcpp::Node
{
public:
  CupHolderDetectorNode();

private:
  CupHolderDetectorConfig loadConfigFromParams() const;

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  /// Caches msg as latest_marker_overlay_ — see class doc comment. Does
  /// no detection work itself.
  void markerOverlayCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

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
  /// Subscribed only when config_.publish_overlay_image is true — see
  /// markerOverlayCallback.
  image_transport::Subscriber marker_overlay_sub_;
  /// The combined (marker + cup_holder/hole) overlay — see class doc
  /// comment. Same topic name real's aruco_detector_node/
  /// yolo_marker_bridge_node publish directly; here this node is the sole
  /// publisher.
  image_transport::Publisher overlay_image_pub_;

  /// Latest frame received on overlay_image_input_topic, cached (not
  /// re-published as-is) — see class doc comment. cv_bridge::CvImageConstPtr
  /// so it's a cheap shared_ptr copy per callback, not a deep image copy.
  /// nullptr until aruco_detector_node's first overlay frame arrives.
  cv_bridge::CvImageConstPtr latest_marker_overlay_;
};

}  // namespace aruco_perception

#endif  // ARUCO_PERCEPTION__CUP_HOLDER_DETECTOR_NODE_HPP_
