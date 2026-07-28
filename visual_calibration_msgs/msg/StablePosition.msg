# One tracked instance's (cup_holder, or a specific hole) held, drift-
# filtered position, published by depth_perception_node — see
# StablePositionArray.msg for the array wrapper and its publish-rate
# rationale. This is the OUTPUT of depth_perception's rolling-window +
# "hold last known position" filtering (see depth_perception_node.hpp's
# RollingWindow/last_stable), not raw per-frame YOLO output like
# Detection2D — a consumer of THIS message never sees the flicker a raw
# per-frame stream would have, since it's published continuously (every
# detections_2d callback) for every instance ever successfully detected,
# regardless of whether that instance appeared in the triggering frame.

# "cup_holder" or "hole" — same convention as Detection2D.msg.
string class_name

# ONLY meaningful for class_name == "hole" (1-4, fixed image-space
# quadrant) — 0/unset for "cup_holder". Same convention as
# Detection2D.msg's hole_number.
int32 hole_number

# 3D position, camera's own optical frame (meters, right-handed, +Z out
# of the lens) — same frame convention as aruco_detector_node's
# marker_pose. This is depth_perception's held last_stable position, NOT
# a single frame's raw back-projected reading.
float64 x
float64 y
float64 z

# The SAME position as (x, y, z) above, reprojected back into 2D pixel
# coordinates via the camera's own intrinsics — computed by
# depth_perception_node specifically so a consumer that only wants to
# DRAW this position on an image (e.g. yolo_marker_bridge_node's overlay)
# never needs its own camera-frame/pinhole-projection math; it can treat
# (px, py) as opaque "where to draw this" data. Always consistent with
# (x, y, z) — both computed from the same held position, same intrinsics.
float64 px
float64 py

# True if THIS message's position was just updated (a genuine drift past
# depth_perception's stable_drift_threshold_m), false if it's still
# holding a previously-established position unchanged this callback — see
# RollingWindow::updateLastStable's doc comment.
bool drifted

# Number of samples currently in this instance's rolling window (see
# DepthPerceptionConfig::rolling_window_size) — a rough confidence/
# maturity signal only, not meant for precise consumption.
int32 sample_count
