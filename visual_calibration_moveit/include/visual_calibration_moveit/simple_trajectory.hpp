#ifndef VISUAL_CALIBRATION_MOVEIT__SIMPLE_TRAJECTORY_HPP_
#define VISUAL_CALIBRATION_MOVEIT__SIMPLE_TRAJECTORY_HPP_

#include <array>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace visual_calibration_moveit
{

/// Computes a goal pose standoff_m in front of camera_tf, along camera_tf's
/// own local +Z axis (the REP-103 optical-frame forward convention), then
/// rotated by facing_rpy_rad (roll, pitch, yaw, in radians, applied in
/// camera_tf's own local frame) to control how the goal's axes relate to
/// the camera's axes — e.g. (pi/2, pi/2, 0) swaps X<->Y and flips Z, so the
/// goal's Z ends up facing back toward the camera. This rotation encodes a
/// facing preference (a design choice, not something measurable from any
/// TF), so it is a parameter here rather than a value computed from TF —
/// see simple_trajectory_sim.yaml/_real.yaml.
/// camera_tf must already be expressed in the frame the caller wants the
/// resulting Pose in (e.g. the planning frame) — this function does no
/// further frame conversion. This is the pose for whatever frame should
/// end up facing the camera this way (e.g. the ArUco marker link) — not
/// necessarily the link MoveIt actually commands; see
/// planAndExecuteInFrontOf for that conversion.
geometry_msgs::msg::Pose offsetInFrontOf(
  const geometry_msgs::msg::TransformStamped & camera_tf,
  double standoff_m,
  const std::array<double, 3> & facing_rpy_rad);

/// Per-environment (sim/real) tuning for planAndExecuteInFrontOf, meant to
/// be loaded from a parameter file — sim and real need different values
/// since the real robot's geometry isn't calibrated yet (different camera
/// frame name, mounting, standoff, facing). Room to grow (tolerances,
/// retries, waypoint offsets) as the recalibration workflow is built out
/// in later tasks.
struct StandoffConfig
{
  /// TF frame to stand off in front of, e.g. the camera's optical frame.
  std::string camera_frame;
  /// TF frame to place at the standoff pose — must be part of the
  /// planning group's kinematic chain (see aruco_moveit_config's SRDF).
  std::string end_effector_frame;
  /// Distance in front of camera_frame to place the goal pose.
  double standoff_m = 0.0;
  /// Datasheet reach cap: reject rather than plan if the resulting pose
  /// would be farther than this from the planning frame's origin.
  double max_reach_m = 0.0;
  /// Roll, pitch, yaw (radians) applied in the camera frame's own local
  /// axes to compute the goal orientation — see offsetInFrontOf.
  std::array<double, 3> facing_rpy_rad = {0.0, 0.0, 0.0};
};

/// One-shot trajectory generation via MoveGroupInterface: set a target
/// pose (e.g. derived from the camera->base_link TF chain), plan, execute.
/// No staged approach/retreat/gripper choreography — see MtcTrajectory
/// for that.
///
/// Holds (rather than inherits) the rclcpp::Node, matching
/// MoveGroupInterface's own (node, group_name) constructor and avoiding
/// shared_from_this() pitfalls.
class SimpleTrajectory
{
public:
  explicit SimpleTrajectory(
    const rclcpp::Node::SharedPtr & node,
    const std::string & planning_group = "ur_manipulator");

  /// Plan and execute a trajectory to the given target pose
  /// (in the MoveGroupInterface's planning frame). Returns true on
  /// successful plan + execute.
  bool planAndExecute(const geometry_msgs::msg::Pose & target_pose);

  /// Looks up config.camera_frame, computes a pose config.standoff_m in
  /// front of it (see offsetInFrontOf), then plans + executes so
  /// config.end_effector_frame reaches that pose — via
  /// setEndEffectorLink(config.end_effector_frame), so that frame must be
  /// part of the planning group's kinematic chain (see aruco_moveit_config's
  /// SRDF). Refuses to plan if the resulting pose is farther than
  /// config.max_reach_m from the planning frame's origin.
  /// tf_timeout bounds how long the TF lookup waits for config.camera_frame
  /// to become available (TransformListener needs a moment after
  /// construction to receive /tf_static).
  bool planAndExecuteInFrontOf(
    const StandoffConfig & config,
    rclcpp::Duration tf_timeout = rclcpp::Duration::from_seconds(3.0));

  /// Same as the above, using the StandoffConfig loaded from parameters at
  /// construction time (see loadStandoffConfigFromParams).
  bool planAndExecuteInFrontOf(
    rclcpp::Duration tf_timeout = rclcpp::Duration::from_seconds(3.0));

private:
  /// Reads camera_frame, end_effector_frame, standoff_m, max_reach_m, and
  /// facing_rpy_rad (a 3-element array) from this node's declared
  /// parameters and returns them as a StandoffConfig. Requires the node to
  /// have been started with a parameter file providing all five (e.g. via
  /// automatically_declare_parameters_from_overrides).
  StandoffConfig loadStandoffConfigFromParams() const;

  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface move_group_interface_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  StandoffConfig standoff_config_;
};

}  // namespace visual_calibration_moveit

#endif  // VISUAL_CALIBRATION_MOVEIT__SIMPLE_TRAJECTORY_HPP_
