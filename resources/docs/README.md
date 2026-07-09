# Visual Calibration — Documentation

## Introduction

Reach out and touch your nose with your eyes closed. You can do it, roughly,
because your brain already knows where your hand is relative to your body —
but *seeing* your hand land exactly on target is a different kind of
knowledge, one your eyes provide by relating what they observe back to your
body's own sense of itself. That relationship between "what my eyes see" and
"where my hand actually is" isn't hardwired; it's learned and continuously
recalibrated, which is why hand-eye coordination briefly falls apart when you
put on someone else's glasses.

A robot arm has the equivalent of body-awareness for free: joint encoders and
forward kinematics mean the arm always knows exactly where its own end
effector is, expressed in its `base_link` frame. A camera watching that arm
has none of that. It can see the arm, but it has no built-in notion of where
it itself is sitting relative to `base_link` — that link is simply missing
from the robot's frame tree. This project's job is to establish that missing
`camera → base_link` transform automatically, the same way biological hand-eye
coordination is calibrated: by showing the camera a known reference — an
ArUco marker rigidly mounted on the arm's own end effector — and letting the
camera work out where it is relative to the robot from that.

**Why simulation first:** in the Starbots Cafeteria Gazebo simulation, Gazebo
already publishes `base_link → wrist_rgbd_camera_depth_optical_frame` as
ground truth (it placed the camera model itself). The real robot has no such
ground truth. Simulation is therefore used to validate the calibration
pipeline's computed transform against a known-correct answer before ever
trusting it on real hardware.

## Table of contents

- [Architecture](./architecture.md) — project/package layout, dependency on
  the wider workspace, and the data-flow pipeline from camera image to
  broadcast TF.
- [Calibration process](./calibration_process.md) — plain-language,
  step-by-step walkthrough of what happens during a `~/calibrate` run,
  including what a "sample" is and what the `mean_spread_deg`/
  `max_spread_deg` result fields mean.
- [aruco_perception](./aruco_perception.md) — the marker detection and
  TF-chaining nodes.
- [visual_calibration_moveit](./visual_calibration_moveit.md) — the MoveIt2
  interaction nodes (planning scene setup, trajectory planning, and the
  MoveIt Task Constructor node).
- [aruco_moveit_config](./aruco_moveit_config.md) — this project's MoveIt2
  config for the UR3e + RG2 gripper, and specifically the one deliberate
  change made to it (the `tip_link` change in the SRDF).
- [calibration_validation](./calibration_validation.md) — the sim-only node
  that automatically checks the broadcast calibration TF against
  simulation's own ground-truth camera TF.

## Class-level docs

A separate, per-class documentation set (Mermaid class diagrams + one
section per public method), piloted on `visual_calibration_moveit` and now
covering every package with hand-written classes. Start at
[class_docs/README.md](./class_docs/README.md) for the full index and a
package/class-level "who talks to whom" diagram; see
[class_docs/CONVENTIONS.md](./class_docs/CONVENTIONS.md) for the
documentation conventions used throughout.

- [class_docs/aruco_perception.md](./class_docs/aruco_perception.md)
- [class_docs/visual_calibration_moveit.md](./class_docs/visual_calibration_moveit.md)
- [class_docs/visual_calibration_msgs.md](./class_docs/visual_calibration_msgs.md)