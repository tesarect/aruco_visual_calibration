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
| `publish_overlay_image` | bool | `true` | Whether to draw and publish a debug image with the detected marker's border and axes. |
| `overlay_image_topic` | string | `/aruco_perception/overlay_image` | Topic the overlay image is published on when enabled. |
| `overlay_border_color_bgr` | int[3] | `[0, 255, 255]` | BGR (not RGB) color OpenCV draws the marker's detected border in — default is yellow. |

## Marker identity (known/given)

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `marker_length_m` | double | `0.045` | Physical side length of the marker in meters — must match the real marker exactly, since it directly scales the estimated pose's translation. |
| `dictionary_name` | string | `DICT_4X4_50` | Which OpenCV predefined ArUco dictionary to match candidates against. One of `DICT_4X4_50`, `DICT_4X4_100`, `DICT_4X4_250`, `DICT_4X4_1000` (see `dictionaryFromName`); any other value throws at startup. |
| `marker_id` | int | `0` | The single marker ID this node looks for in each frame. Markers with other IDs in view are ignored. |

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
