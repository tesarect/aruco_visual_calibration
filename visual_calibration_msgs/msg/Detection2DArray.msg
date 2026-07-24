# Published continuously by yolo_marker_bridge_node on
# /aruco_perception/detections_2d, every processed camera frame — same
# publish cadence and QoS (plain reliable, depth-10 queue) as
# /aruco_perception/marker_pose, NOT transient_local/latched. This is a
# derived detection stream depth-perception filters/votes over across
# multiple frames on its own side (single-frame depth is noisy, especially
# near a cavity rim), not a one-shot/on-demand sample — so it needs a
# running stream to accumulate over, same reasoning marker_pose already
# follows for its own continuous publish.
#
# Carries BOTH cup_holder and hole detections in one array (distinguished
# by each Detection2D's class_name) rather than two separate topics — one
# subscription for depth-perception to filter on class_name itself.
#
# NOTE: deliberately NOT vision_msgs/Detection2DArray — vision_msgs is not
# a dependency anywhere in this workspace, and its Detection2D is
# bounding-box-and-hypothesis-centric with pose/covariance fields this
# project doesn't need. Named identically to vision_msgs' types on purpose
# for readability, but they are NOT interchangeable — always import from
# visual_calibration_msgs here, not vision_msgs.

# Same image frame_id/stamp convention as marker_pose: reuse the source
# Image message's own header, not now().
std_msgs/Header header

Detection2D[] detections