[← Back to index](./README.md)

# image_subscriber_sim.yaml — parameter reference

Parameters for `image_subscriber_node`, loaded under its `ros__parameters`
namespace. See [aruco_perception.md](./aruco_perception.md) — this is the
plumbing-only smoke-test node, not the ArUco detector.

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `image_topic` | string | `/wrist_rgbd_depth_sensor/image_raw` | Camera image topic to subscribe to and confirm is arriving. |
| `camera_info_topic` | string | `/wrist_rgbd_depth_sensor/camera_info` | Camera intrinsics topic to subscribe to and confirm is arriving. |
