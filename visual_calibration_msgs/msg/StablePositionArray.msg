# Published continuously by depth_perception_node on
# /depth_perception/stable_positions, every processed detections_2d
# callback — one StablePosition per tracked instance EVER successfully
# detected so far (cup_holder, or each hole_1..hole_4 seen at least once),
# regardless of whether that instance appeared in the triggering frame's
# detections_2d message. This is deliberate: the whole point of this topic
# is a continuous, gap-free stream a consumer (e.g. a future camera-
# >base_link TF broadcaster, or yolo_marker_bridge_node's overlay image)
# can rely on always having a usable position, instead of needing to
# handle "this instance vanished for a few frames" itself. Same publish-
# continuously-even-when-nothing-changed reasoning as
# Detection2DArray.msg/marker_pose already follow.
#
# NOT the same as Detection2DArray — that's raw, per-frame, unfiltered
# YOLO output; this is depth_perception's held/filtered output built FROM
# that stream. A consumer that wants raw detections still subscribes to
# Detection2DArray; a consumer that wants a stable, always-available
# position subscribes to this instead.

# Same image frame_id/stamp convention as Detection2DArray/marker_pose:
# reuse the source Image message's own header, not now(). Note: since this
# array can include instances that did NOT appear in the frame that
# triggered this publish, the stamp reflects when this message was
# published, not necessarily when every included position was last
# actually re-detected — sample_count/drifted on each StablePosition are
# the per-instance freshness signals, not this header.
std_msgs/Header header

StablePosition[] positions
