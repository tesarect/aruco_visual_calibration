[← Back to index](./README.md)

# aruco_detector_sim.yaml — parameter reference

Parameters for `aruco_detector_node`, loaded under its `ros__parameters`
namespace. See [aruco_perception.md](./aruco_perception.md) for the node
itself.

## Topics

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `image_topic` | string | `/wrist_rgbd_depth_sensor/image_raw` | Camera image topic to subscribe to. |
| `camera_info_topic` | string | `/wrist_rgbd_depth_sensor/camera_info` | Camera intrinsics topic; needed before pose estimation can run. |
| `pose_topic` | string | `/aruco_perception/marker_pose` | Where the detected marker's `PoseStamped` (camera → marker) is published. |

## Overlay

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `publish_overlay_image` | bool | `true` | Whether to draw and publish a debug image with the detected marker's border and axes, on every processed frame (not just frames where the marker was found). |
| `overlay_image_topic` | string | sim: `/aruco_perception/overlay_image_marker_only`, real: `/aruco_perception/overlay_image` | Topic the marker-only overlay image is published on when enabled. **Sim only**, this is an intermediate topic, not the web-facing one: `cup_holder_detector_node` subscribes here, draws `cup_holder`/`hole` on top, and republishes the combined image as the sole publisher of `/aruco_perception/overlay_image` — matching real's existing single-publisher pattern (`yolo_marker_bridge_node` already draws marker+cup_holder+hole together before its own single publish). Real is unchanged: `aruco_detector_real.yaml` publishes directly to `/aruco_perception/overlay_image`, since `cup_holder_detector_node` never runs there. |
| `overlay_border_color_bgr` | int[3] | `[0, 255, 255]` | BGR (not RGB) color OpenCV draws the marker's detected border in — default is yellow. |
| `detections_2d_topic` | string | `/aruco_perception/detections_2d` | Where this node publishes the marker's pixel-space centroid/bbox as a `Detection2D` (`class_name` `"aruco_marker"`) when found — the same topic `aruco_perception_yolo_bridge`'s `yolo_marker_bridge_node` publishes `cup_holder`/`hole` detections on. |
| `show_centering_crosshair` | bool | `false` | Draws a crosshair at the image's own pixel center in the overlay stream while true. Live-toggled (no restart) by `calibration_orchestrator_node` for the duration of its image-based centering routine — see [../orchestrator.md](../orchestrator.md). |

## Marker identity (known/given)

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `marker_length_m` | double | `0.045` | Physical side length of the marker in meters — must match the real marker exactly, since it directly scales the estimated pose's translation. |
| `dictionary_name` | string | `DICT_4X4_50` | Which OpenCV predefined ArUco dictionary to match candidates against. One of `DICT_4X4_50`, `DICT_4X4_100`, `DICT_4X4_250`, `DICT_4X4_1000` (see `dictionaryFromName`); any other value throws at startup. |
| `marker_id` | int | `0` | The single marker ID this node looks for in each frame. Markers with other IDs in view are ignored. |
| `active` | bool | `true` | Startup default for the classical/hybrid detector switch — `true` means this node is the one actually publishing `marker_pose` on startup. Re-read live (never cached) every frame, so `calibration_orchestrator_node`'s `~/set_detector_mode` takes effect immediately, no restart. See [../orchestrator.md](../orchestrator.md). |

## Detection tuning

These map directly onto OpenCV's `cv::aruco::DetectorParameters`. Sim
lighting is controlled and consistent, so the defaults below are just
OpenCV's own out-of-the-box defaults — expect these to need retuning for
the real robot's less consistent lighting (see the file header comment
in the live YAML).

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `adaptive_thresh_win_size_min` | int | `3` | Smallest adaptive-threshold window size (pixels) tried when binarizing the image to find marker candidates. |
| `adaptive_thresh_win_size_max` | int | `23` | Largest adaptive-threshold window size tried. |
| `adaptive_thresh_win_size_step` | int | `10` | Step size between the min and max window sizes above. |
| `adaptive_thresh_constant` | double | `7.0` | Constant subtracted from the local mean during adaptive thresholding. |
| `min_marker_perimeter_rate` | double | `0.03` | Minimum candidate marker perimeter, as a fraction of the image's largest dimension — filters out tiny false-positive squares. |
| `corner_refinement_method` | int (enum) | `1` | Which OpenCV corner-refinement algorithm to run after initial detection: `0` = none, `1` = subpixel, `2` = contour, `3` = AprilTag-style. Default (`1`, subpixel) trades a little CPU time for more accurate corner localization, which matters since corner accuracy directly affects the estimated pose. |
| `corner_refinement_win_size` | int | `5` | Pixel-radius neighborhood searched around each initial corner guess during refinement (OpenCV default). Added 2026-08-03 after observing non-square/non-parallelogram corners in the overlay on real; a real-only tuning pass swept this alongside the 3 fields below it. |
| `corner_refinement_max_iterations` | int | `30` | Refinement stop criterion: max iterations before giving up (OpenCV default). |
| `corner_refinement_min_accuracy` | double | `0.1` | Refinement stop criterion: minimum error to consider converged (OpenCV default). |
| `polygonal_approx_accuracy_rate` | double | `0.03` | How loosely a candidate contour is approximated as a quadrilateral before its 4 corners are extracted, upstream of refinement entirely (OpenCV default). Too loose is a plausible direct cause of a corner visibly pulled in/out of an otherwise-square marker. |

A per-frame corner-squareness diagnostic (side/diagonal lengths of the
detected quadrilateral, throttled log) exists specifically to tune the 4
params above against real hardware — see `aruco_detector_node.cpp`'s
`imageCallback`.
