# A single YOLO-detected cup_holder/hole instance, 2D pixel space only —
# NOT a 3D pose. Unlike the ArUco marker (known 45mm real-world size,
# solvable via solvePnP), cup_holder/hole have no known real-world circle
# size to solve a 3D pose against from YOLO alone. depth-perception is the
# intended consumer: it looks up depth at (cx, cy) — using bbox to sample a
# small neighborhood/patch rather than a single noisy pixel, especially
# near a cavity rim — and back-projects to 3D itself using the depth
# camera's own intrinsics. See Detection2DArray.msg for the array wrapper
# and its publish-rate/QoS rationale.

# "cup_holder" or "hole" — matches YOLO-pipeline's dataset/data.yaml class
# names exactly (not an enum — keeps this .msg stable if a class is ever
# renamed/added on the YOLO side without a message-shape change here).
string class_name

# Pixel coordinates of the detection's centroid (bbox center) in the
# source image. For a tight axis-aligned box on a straight-on circular
# object, this already equals the circle's true 2D center — see
# YOLO-pipeline/aruco_pose.py's detect_centroids() docstring for the full
# reasoning/caveats.
float64 cx
float64 cy

float64 confidence

# [x1, y1, x2, y2] in pixels — the detection's bounding box, included so
# depth-perception can sample a small patch within it rather than a single
# pixel (more robust against invalid/noisy single-pixel depth reads).
float64[4] bbox