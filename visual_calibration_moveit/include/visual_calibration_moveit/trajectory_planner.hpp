#ifndef VISUAL_CALIBRATION_MOVEIT__TRAJECTORY_PLANNER_HPP_
#define VISUAL_CALIBRATION_MOVEIT__TRAJECTORY_PLANNER_HPP_

#include <array>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visual_calibration_msgs/msg/planning_failure.hpp>
#include <visual_calibration_msgs/srv/get_polygon_waypoints.hpp>
#include <visual_calibration_msgs/srv/get_preset_pose.hpp>
#include <visual_calibration_msgs/srv/get_standoff_pose.hpp>
#include <visual_calibration_msgs/srv/trace_path.hpp>

#include "visual_calibration_moveit/preset_poses.hpp"

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

/// Tuning for the sequenced-goal stay/lift/standby behavior (see
/// ArmState), loaded from a parameter file alongside StandoffConfig/
/// PolygonConfig.
struct SequenceConfig
{
  /// How long to stay AT a sequenced goal (e.g. a hole/cupholder pose)
  /// before automatically lifting away from it.
  double stay_seconds_at_goal = 4.0;
  /// Absolute Z coordinate (in the planning frame, i.e. base_link's own
  /// Z — 0.0 means level with base_link's origin) the arm lifts to after
  /// stay_seconds_at_goal, keeping the goal's X/Y/orientation unchanged.
  /// NOT a relative offset added to the goal's Z — an absolute target
  /// height, per project convention ("lift to roughly base_link's Z
  /// plane").
  double lift_target_z_m = 0.0;
  /// How long to wait at the lifted pose, with no new sequenced goal
  /// arriving, before automatically moving to the "standby" preset.
  double lift_wait_seconds = 8.0;
};

/// TrajectoryPlanner has no built-in understanding of the robot's task —
/// it doesn't know what "calibration" or "inspection" means, only "move to
/// this pose". This enum is the small amount of bookkeeping it keeps about
/// its OWN recent activity, so it can automatically stay-then-lift-then-
/// standby after a "sequenced goal" (see TracePath.srv's is_sequenced_goal
/// field) without a caller having to drive every step of that dance
/// itself. In plain terms: the node remembers "did I just visit a regular
/// goal, and if so, what pose was that", and runs two timers off of that
/// one fact — it is not reasoning about the overall workflow, just
/// reacting mechanically to a flag it's told. Unlike an earlier version of
/// this design, it does NOT return to wherever the arm happened to be
/// before the goal — every sequenced goal always resolves to the SAME two
/// destinations (a lift straight up from the goal, then "standby"), never
/// an arbitrary prior pose.
enum class ArmState
{
  /// No sequenced goal is pending stay/lift/standby handling — this is
  /// also the state right after startup/home/cal_ready moves, which never
  /// trigger this machinery (see is_sequenced_goal doc comment).
  IDLE,
  /// Just reached a sequenced goal; waiting out stay_seconds_at_goal
  /// before lifting straight up from it.
  SETTLED_AT_GOAL,
  /// Lifted to Z = lift_target_z_m; waiting out lift_wait_seconds with no
  /// new sequenced goal before moving to "standby".
  LIFTED_IDLE,
  /// At the "standby" preset — stays here until the next sequenced goal.
  STANDBY,
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

  /// Computes the standoff pose (see offsetInFrontOf) from the configured
  /// StandoffConfig WITHOUT moving the arm. If the camera_frame TF lookup
  /// fails, falls back to the "standoff" entry in preset_poses_ (see
  /// PresetPoses) — used_fallback in the returned pair distinguishes which
  /// source was used. Returns std::nullopt only if NEITHER a live TF
  /// lookup NOR a "standoff" preset was available.
  std::optional<std::pair<geometry_msgs::msg::Pose, bool /*used_fallback*/>> getStandoffPose(
    rclcpp::Duration tf_timeout = rclcpp::Duration::from_seconds(3.0)) const;

  /// Returns the named preset's pose (see PresetPoses) WITHOUT moving the
  /// arm. Returns std::nullopt if no preset with that name was loaded.
  std::optional<geometry_msgs::msg::Pose> getPresetPose(const std::string & name) const;

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

  /// Handles a GetStandoffPose service request by calling
  /// getStandoffPose() and returning the result — no motion.
  void handleGetStandoffPose(
    const std::shared_ptr<visual_calibration_msgs::srv::GetStandoffPose::Request> request,
    std::shared_ptr<visual_calibration_msgs::srv::GetStandoffPose::Response> response);

  /// Handles a GetPresetPose service request by calling getPresetPose()
  /// and returning the result — no motion.
  void handleGetPresetPose(
    const std::shared_ptr<visual_calibration_msgs::srv::GetPresetPose::Request> request,
    std::shared_ptr<visual_calibration_msgs::srv::GetPresetPose::Response> response);

  /// Reads camera_frame, end_effector_frame, standoff_m, max_reach_m, and
  /// facing_rpy_rad (a 3-element array) from this node's declared
  /// parameters and returns them as a StandoffConfig. Requires the node to
  /// have been started with a parameter file providing all five (e.g. via
  /// automatically_declare_parameters_from_overrides).
  StandoffConfig loadStandoffConfigFromParams() const;

  /// Reads polygon_num_corners and polygon_radius_m from this node's
  /// declared parameters and returns them as a PolygonConfig.
  PolygonConfig loadPolygonConfigFromParams() const;

  /// Reads stay_seconds_at_goal, lift_target_z_m, and lift_wait_seconds from
  /// this node's declared parameters and returns them as a SequenceConfig.
  SequenceConfig loadSequenceConfigFromParams() const;

  /// Called after a successful is_sequenced_goal move (see handleTracePath).
  /// Cancels any pending stay/lift timer, transitions to
  /// ArmState::SETTLED_AT_GOAL, and (re)starts stay_timer_ for
  /// sequence_config_.stay_seconds_at_goal, at the end of which
  /// onStayTimerFired runs. goal_pose is the pose the arm just reached —
  /// stay_timer_'s lift is computed from THIS, not from any pose the arm
  /// was at before the goal (this design does not track/return-to prior
  /// poses at all, see ArmState's doc comment).
  void onSequencedGoalReached(const geometry_msgs::msg::Pose & goal_pose);

  /// Fires once stay_timer_ elapses. Plans and executes to goal_pose_ with
  /// Z set to sequence_config_.lift_target_z_m (same X/Y/orientation,
  /// absolute Z target — see SequenceConfig::lift_target_z_m). On success,
  /// transitions to ArmState::LIFTED_IDLE and starts lift_wait_timer_ for
  /// sequence_config_.lift_wait_seconds (see onLiftWaitTimerFired). On
  /// failure, reports via publishPlanningFailure (context "lift") and
  /// returns to ArmState::IDLE — does NOT proceed to standby from a failed
  /// lift, since the arm's actual position at that point is uncertain.
  void onStayTimerFired();

  /// Fires once lift_wait_timer_ elapses with no new sequenced goal having
  /// arrived in the meantime (a new sequenced goal cancels this timer via
  /// onSequencedGoalReached instead). Plans and executes to the "standby"
  /// preset; on success transitions to ArmState::STANDBY and publishes
  /// "standby" via publishCurrentPoseName. On failure (missing preset or
  /// plan/execute failure), reports via publishPlanningFailure (context
  /// "standby") and returns to ArmState::IDLE.
  void onLiftWaitTimerFired();

  /// Called once from the constructor. If move_to_home_on_startup (a
  /// declared param, see trajectory_planner_sim.yaml/_real.yaml) is true,
  /// plans and executes to the "home" preset pose — an explicit, opt-in
  /// reversal of this node's original "never move on startup" design (see
  /// main.cpp's history / todo.txt item 1); the param makes the choice to
  /// auto-move an auditable config decision rather than silent behavior.
  /// On failure (missing "home" preset, or plan/execute failure), logs the
  /// error and reports it via ~/planning_failure (see PlanningFailure.msg)
  /// — does not throw, does not block node startup either way.
  void runStartupSequence();

  /// Publishes name on ~/current_pose_name (transient_local, so a late
  /// subscriber — e.g. the web bridge reconnecting — immediately gets the
  /// last-published value instead of nothing). Called after every
  /// successful move that lands on a known named pose. Not called on
  /// arbitrary/unnamed trace_path waypoints (e.g. calibration polygon
  /// corners) — those aren't meaningful to show a human as a "current
  /// pose name".
  void publishCurrentPoseName(const std::string & name);

  /// Publishes one PlanningFailure message on ~/planning_failure — an
  /// event, not a state, so plain reliable QoS (no transient_local): a
  /// failure that already happened has no "current value" worth replaying
  /// to a late subscriber. context is a short machine-readable label (e.g.
  /// "startup_home"), message is the human-readable reason. Also logs via
  /// RCLCPP_ERROR at the call site — this only handles the web-facing
  /// side.
  void publishPlanningFailure(const std::string & context, const std::string & message);

  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface move_group_interface_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  StandoffConfig standoff_config_;
  PolygonConfig polygon_config_;
  SequenceConfig sequence_config_;
  PresetPoses preset_poses_;
  rclcpp::Service<visual_calibration_msgs::srv::TracePath>::SharedPtr trace_path_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trace_polygon_service_;
  rclcpp::Service<visual_calibration_msgs::srv::GetPolygonWaypoints>::SharedPtr
    get_polygon_waypoints_service_;
  rclcpp::Service<visual_calibration_msgs::srv::GetStandoffPose>::SharedPtr
    get_standoff_pose_service_;
  rclcpp::Service<visual_calibration_msgs::srv::GetPresetPose>::SharedPtr
    get_preset_pose_service_;
  /// transient_local: late subscribers get the last-published name
  /// immediately instead of waiting for the next state change. Event-driven
  /// (published only on transitions), not on a timer — does not add
  /// periodic DDS traffic alongside perception topics.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr current_pose_name_pub_;
  /// Plain reliable QoS, published once per failure event — see
  /// publishPlanningFailure's doc comment for why this is NOT
  /// transient_local (unlike current_pose_name_pub_).
  rclcpp::Publisher<visual_calibration_msgs::msg::PlanningFailure>::SharedPtr
    planning_failure_pub_;

  /// Guards arm_state_/goal_pose_/stay_timer_/lift_wait_timer_ below.
  /// Required since main.cpp switched to a MultiThreadedExecutor (see its
  /// comment) — service callbacks (handleTracePath) and timer callbacks
  /// (onStayTimerFired/onLiftWaitTimerFired) can now genuinely run on
  /// different threads concurrently, e.g. a new sequenced goal arriving
  /// right as a pending timer fires.
  std::mutex state_mutex_;
  /// See ArmState's doc comment — this class's only memory of its own
  /// recent activity, used solely to drive the sequenced-goal stay/lift/
  /// standby dance. Starts IDLE; runStartupSequence's home move does not
  /// touch this (only is_sequenced_goal calls do). Guarded by
  /// state_mutex_.
  ArmState arm_state_ = ArmState::IDLE;
  /// The most recently reached sequenced goal's pose — only meaningful
  /// while arm_state_ is SETTLED_AT_GOAL or LIFTED_IDLE. The lift pose
  /// (see onStayTimerFired) is computed from this, not from any pose the
  /// arm was at before the goal. Guarded by state_mutex_.
  geometry_msgs::msg::Pose goal_pose_;
  /// One-shot timer for SequenceConfig::stay_seconds_at_goal — see
  /// onSequencedGoalReached/onStayTimerFired. Guarded by state_mutex_.
  rclcpp::TimerBase::SharedPtr stay_timer_;
  /// One-shot timer for SequenceConfig::lift_wait_seconds — see
  /// onStayTimerFired/onLiftWaitTimerFired. Cancelled and restarted by any
  /// new sequenced goal that arrives while pending (see
  /// onSequencedGoalReached) — this cancel-and-restart IS the "idle"
  /// signal: the timer only ever fires if nothing new showed up in time.
  /// Guarded by state_mutex_.
  rclcpp::TimerBase::SharedPtr lift_wait_timer_;
};

}  // namespace visual_calibration_moveit

#endif  // VISUAL_CALIBRATION_MOVEIT__TRAJECTORY_PLANNER_HPP_
