#ifndef VISUAL_CALIBRATION_MOVEIT__TRAJECTORY_PLANNER_HPP_
#define VISUAL_CALIBRATION_MOVEIT__TRAJECTORY_PLANNER_HPP_

#include <array>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visual_calibration_msgs/srv/get_polygon_waypoints.hpp>
#include <visual_calibration_msgs/srv/trace_path.hpp>

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
/// see trajectory_planner_sim.yaml/_real.yaml.
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

/// Tuning for polygonWaypointsAroundStandoff, loaded from a parameter file
/// alongside StandoffConfig. num_corners selects the shape (3=triangle,
/// 4=square, 5=pentagon, ...); radius_m is each corner's distance from the
/// standoff pose's center, in its own local X/Y plane.
struct PolygonConfig
{
  /// Number of corners; must be >= 3.
  int num_corners = 4;
  /// Distance from center to each corner, in the standoff pose's local X/Y
  /// plane. Keep small relative to StandoffConfig::max_reach_m.
  double radius_m = 0.05;
  /// Default planning_mode used by ~/trace_polygon (a plain Trigger, so it
  /// has no per-call field for this) — one of
  /// TracePath::Request::PLANNING_MODE_JOINT_SPACE/PLANNING_MODE_CARTESIAN.
  /// Callers that need to choose per-call (e.g. calibration_broadcaster_node)
  /// use ~/trace_path directly, which takes planning_mode as a request field.
  uint8_t default_planning_mode =
    visual_calibration_msgs::srv::TracePath::Request::PLANNING_MODE_CARTESIAN;
};

/// Trajectory generation via MoveGroupInterface: set a target pose (e.g.
/// derived from the camera->base_link TF chain), plan, execute. No staged
/// approach/retreat/gripper choreography — see MtcTrajectory for that.
///
/// Holds (rather than inherits) the rclcpp::Node, matching
/// MoveGroupInterface's own (node, group_name) constructor and avoiding
/// shared_from_this() pitfalls.
class TrajectoryPlanner
{
public:
  explicit TrajectoryPlanner(
    const rclcpp::Node::SharedPtr & node,
    const std::string & planning_group = "ur_manipulator");

  /// Plan and execute a trajectory to the given target pose
  /// (in the MoveGroupInterface's planning frame), via free-space
  /// joint-space planning (MoveGroupInterface::plan()/execute()) — no
  /// straight-line guarantee on the path shape, but generally more likely
  /// to succeed than planAndExecuteCartesian() near limits/obstacles.
  /// Returns true on successful plan + execute.
  bool planAndExecute(const geometry_msgs::msg::Pose & target_pose);

  /// Plan and execute a straight-line Cartesian path from the current pose
  /// to target_pose, via MoveGroupInterface::computeCartesianPath().
  /// Collision-aware (checked against the planning scene, same as
  /// planAndExecute). Fails (returns false, does not execute anything) if
  /// the achieved fraction is below min_fraction — executing a partial
  /// Cartesian path would stop short of target_pose, at an undefined
  /// intermediate point, which is unsafe to treat as "the waypoint" for
  /// calibration sampling. No automatic fallback to joint-space planning
  /// on an incomplete path (see progress.md's Feature Additions for that
  /// as a possible future addition) — callers needing that today should
  /// retry via planAndExecute() themselves.
  bool planAndExecuteCartesian(
    const geometry_msgs::msg::Pose & target_pose,
    double min_fraction = 0.95);

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

  /// Visits each pose in waypoints in order, stopping at the first
  /// failure. planning_mode selects planAndExecute() (joint-space) or
  /// planAndExecuteCartesian() (straight-line) for every waypoint-to-
  /// waypoint move — see TracePath::Request::PLANNING_MODE_*. Used to
  /// spread calibration samples across several arm poses — see
  /// polygonWaypointsAroundStandoff for the concrete shape used today.
  /// Returns true only if every waypoint succeeded.
  bool tracePath(
    const std::vector<geometry_msgs::msg::Pose> & waypoints,
    uint8_t planning_mode =
    visual_calibration_msgs::srv::TracePath::Request::PLANNING_MODE_CARTESIAN);

  /// Computes and returns polygonWaypointsAroundStandoff's waypoints
  /// WITHOUT moving the arm — lets a caller (e.g.
  /// calibration_broadcaster_node) drive them one at a time itself via
  /// ~/trace_path, without duplicating this node's standoff/polygon
  /// geometry math or config. Returns an empty vector (and logs the
  /// error) if the camera TF lookup fails — same failure mode as
  /// polygonWaypointsAroundStandoff, which this wraps.
  std::vector<geometry_msgs::msg::Pose> getPolygonWaypoints(
    rclcpp::Duration tf_timeout = rclcpp::Duration::from_seconds(3.0)) const;

private:
  /// Computes polygon_config_.num_corners waypoints forming a regular
  /// polygon of radius polygon_config_.radius_m, in the standoff pose's own
  /// local X/Y plane, centered on the standoff pose computed from
  /// standoff_config_ (see planAndExecuteInFrontOf). Every corner keeps the
  /// same facing_rpy_rad-derived orientation as the center — only position
  /// varies — so the target link keeps facing the camera at each corner.
  /// Corners are visited in angular order (not skipping around), so
  /// consecutive waypoints are always adjacent. Returns an empty vector (and
  /// logs the error) if the camera TF lookup fails.
  std::vector<geometry_msgs::msg::Pose> polygonWaypointsAroundStandoff(
    rclcpp::Duration tf_timeout) const;

  /// Handles a TracePath service request by calling tracePath() with the
  /// request's waypoints and planning_mode.
  void handleTracePath(
    const std::shared_ptr<visual_calibration_msgs::srv::TracePath::Request> request,
    std::shared_ptr<visual_calibration_msgs::srv::TracePath::Response> response);

  /// Handles a Trigger service request by computing
  /// polygonWaypointsAroundStandoff and calling tracePath() with them,
  /// using polygon_config_.default_planning_mode (Trigger has no request
  /// fields, so this can't be chosen per-call — see ~/trace_path for that).
  void handleTracePolygon(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /// Handles a GetPolygonWaypoints service request by calling
  /// getPolygonWaypoints() and returning the result — no motion.
  void handleGetPolygonWaypoints(
    const std::shared_ptr<visual_calibration_msgs::srv::GetPolygonWaypoints::Request> request,
    std::shared_ptr<visual_calibration_msgs::srv::GetPolygonWaypoints::Response> response);

  /// Reads camera_frame, end_effector_frame, standoff_m, max_reach_m, and
  /// facing_rpy_rad (a 3-element array) from this node's declared
  /// parameters and returns them as a StandoffConfig. Requires the node to
  /// have been started with a parameter file providing all five (e.g. via
  /// automatically_declare_parameters_from_overrides).
  StandoffConfig loadStandoffConfigFromParams() const;

  /// Reads polygon_num_corners and polygon_radius_m from this node's
  /// declared parameters and returns them as a PolygonConfig.
  PolygonConfig loadPolygonConfigFromParams() const;

  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface move_group_interface_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  StandoffConfig standoff_config_;
  PolygonConfig polygon_config_;
  rclcpp::Service<visual_calibration_msgs::srv::TracePath>::SharedPtr trace_path_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trace_polygon_service_;
  rclcpp::Service<visual_calibration_msgs::srv::GetPolygonWaypoints>::SharedPtr
    get_polygon_waypoints_service_;
};

}  // namespace visual_calibration_moveit

#endif  // VISUAL_CALIBRATION_MOVEIT__TRAJECTORY_PLANNER_HPP_
