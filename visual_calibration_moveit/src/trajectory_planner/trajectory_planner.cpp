#include "visual_calibration_moveit/trajectory_planner.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace visual_calibration_moveit
{

geometry_msgs::msg::Pose offsetInFrontOf(
  const geometry_msgs::msg::TransformStamped & camera_tf,
  double standoff_m,
  const std::array<double, 3> & facing_rpy_rad)
{
  tf2::Transform camera;
  tf2::fromMsg(camera_tf.transform, camera);

  // Move standoff_m along the camera's own local +Z (REP-103 optical-frame
  // forward convention), then rotate by facing_rpy_rad (in the camera's
  // own local frame) to get the desired goal orientation relative to the
  // camera's orientation. facing_rpy_rad encodes a facing preference (a
  // design choice — how the goal's axes should relate to the camera's
  // axes), not a value measured from any TF, so it's a parameter rather
  // than something computed here — see trajectory_planner_sim.yaml.
  tf2::Quaternion facing_quat;
  facing_quat.setRPY(facing_rpy_rad[0], facing_rpy_rad[1], facing_rpy_rad[2]);
  tf2::Transform offset(facing_quat, tf2::Vector3(0.0, 0.0, standoff_m));
  tf2::Transform goal = camera * offset;

  geometry_msgs::msg::Pose goal_pose;
  goal_pose.position.x = goal.getOrigin().x();
  goal_pose.position.y = goal.getOrigin().y();
  goal_pose.position.z = goal.getOrigin().z();
  goal_pose.orientation = tf2::toMsg(goal.getRotation());
  return goal_pose;
}

TrajectoryPlanner::TrajectoryPlanner(
  const rclcpp::Node::SharedPtr & node,
  const std::string & planning_group)
: node_(node),
  move_group_interface_(node_, planning_group),
  tf_buffer_(node_->get_clock()),
  tf_listener_(tf_buffer_),
  standoff_config_(loadStandoffConfigFromParams()),
  polygon_config_(loadPolygonConfigFromParams()),
  sequence_config_(loadSequenceConfigFromParams()),
  preset_poses_(node_)
{
  // "~/..." resolves to a private, node-namespaced service name (e.g.
  // /trajectory_planner/trace_path), the standard ROS2 convention for a
  // service specific to this node.
  trace_path_service_ = node_->create_service<visual_calibration_msgs::srv::TracePath>(
    "~/trace_path",
    std::bind(
      &TrajectoryPlanner::handleTracePath, this, std::placeholders::_1, std::placeholders::_2));

  trace_polygon_service_ = node_->create_service<std_srvs::srv::Trigger>(
    "~/trace_polygon",
    std::bind(
      &TrajectoryPlanner::handleTracePolygon, this, std::placeholders::_1,
      std::placeholders::_2));

  get_polygon_waypoints_service_ =
    node_->create_service<visual_calibration_msgs::srv::GetPolygonWaypoints>(
    "~/get_polygon_waypoints",
    std::bind(
      &TrajectoryPlanner::handleGetPolygonWaypoints, this, std::placeholders::_1,
      std::placeholders::_2));

  get_standoff_pose_service_ =
    node_->create_service<visual_calibration_msgs::srv::GetStandoffPose>(
    "~/get_standoff_pose",
    std::bind(
      &TrajectoryPlanner::handleGetStandoffPose, this, std::placeholders::_1,
      std::placeholders::_2));

  get_preset_pose_service_ =
    node_->create_service<visual_calibration_msgs::srv::GetPresetPose>(
    "~/get_preset_pose",
    std::bind(
      &TrajectoryPlanner::handleGetPresetPose, this, std::placeholders::_1,
      std::placeholders::_2));

  // transient_local + depth 1: a late subscriber (e.g. the web bridge
  // reconnecting) immediately receives the last-published name instead of
  // waiting for the next state change. Published only on transitions, not
  // on a timer — see publishCurrentPoseName.
  current_pose_name_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "~/current_pose_name",
    rclcpp::QoS(1).transient_local().reliable());

  // Event topic, not a state — plain reliable QoS, no transient_local. See
  // publishPlanningFailure's doc comment.
  planning_failure_pub_ = node_->create_publisher<visual_calibration_msgs::msg::PlanningFailure>(
    "~/planning_failure",
    rclcpp::QoS(10).reliable());

  RCLCPP_INFO(
    node_->get_logger(), "trajectory_planner ready (planning group: '%s')",
    planning_group.c_str());

  runStartupSequence();
}

void TrajectoryPlanner::runStartupSequence()
{
  const bool move_to_home_on_startup =
    node_->get_parameter("move_to_home_on_startup").as_bool();

  if (!move_to_home_on_startup) {
    RCLCPP_INFO(
      node_->get_logger(),
      "move_to_home_on_startup is false — staying at the current pose.");
    return;
  }

  const std::optional<geometry_msgs::msg::Pose> home_pose = preset_poses_.get("home");
  if (!home_pose.has_value()) {
    const std::string message =
      "move_to_home_on_startup is true but no 'home' preset is configured — "
      "staying at the current pose. See preset_poses_sim.yaml/_real.yaml.";
    RCLCPP_ERROR(node_->get_logger(), "%s", message.c_str());
    publishPlanningFailure("startup_home", message);
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "Moving to 'home' pose on startup...");
  // Preset poses are captured against end_effector_frame (e.g.
  // rg2_gripper_aruco_link, see pose_capture.py), not whatever link the
  // SRDF defaults MoveGroupInterface's end effector to — must match
  // tracePath()/planAndExecuteInFrontOf()'s existing pattern, or the
  // target pose gets applied to the wrong link and planning fails/aborts
  // against an unreachable target.
  move_group_interface_.setEndEffectorLink(standoff_config_.end_effector_frame);
  const bool succeeded = planAndExecute(*home_pose);
  if (!succeeded) {
    const std::string message = "Startup move to 'home' pose failed.";
    RCLCPP_ERROR(node_->get_logger(), "%s", message.c_str());
    publishPlanningFailure("startup_home", message);
    return;
  }

  publishCurrentPoseName("home");
}

bool TrajectoryPlanner::planAndExecute(const geometry_msgs::msg::Pose & target_pose)
{
  move_group_interface_.setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const bool planned = static_cast<bool>(move_group_interface_.plan(plan));

  if (!planned) {
    RCLCPP_ERROR(node_->get_logger(), "Planning failed for the given target pose");
    return false;
  }

  const bool executed =
    static_cast<bool>(move_group_interface_.execute(plan));

  if (!executed) {
    RCLCPP_ERROR(node_->get_logger(), "Execution failed for the planned trajectory");
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "Trajectory planned and executed successfully");
  return true;
}

bool TrajectoryPlanner::planAndExecuteCartesian(
  const geometry_msgs::msg::Pose & target_pose,
  double min_fraction)
{
  const std::vector<geometry_msgs::msg::Pose> single_target_waypoint{target_pose};
  moveit_msgs::msg::RobotTrajectory trajectory;

  // eef_step = 0.01m: max distance between consecutive interpolated points
  // along the line — fine enough to catch collisions/limits reliably
  // without excessive planning cost for these short (few-cm) calibration
  // moves. jump_threshold = 0.0 disables the jump-detection heuristic
  // (deprecated/unreliable in recent MoveIt anyway); collision checking
  // (the 4th positional bool, default true) stays on.
  const double fraction = move_group_interface_.computeCartesianPath(
    single_target_waypoint, 0.01, 0.0, trajectory);

  if (fraction < min_fraction) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Cartesian path only achieved %.1f%% of the straight-line path to target "
      "(need >= %.1f%%) — refusing to execute a partial path. See "
      "TrajectoryPlanner::planAndExecuteCartesian's doc comment.",
      fraction * 100.0, min_fraction * 100.0);
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory;
  const bool executed = static_cast<bool>(move_group_interface_.execute(plan));

  if (!executed) {
    RCLCPP_ERROR(node_->get_logger(), "Execution failed for the planned Cartesian trajectory");
    return false;
  }

  RCLCPP_INFO(
    node_->get_logger(), "Cartesian trajectory planned (%.1f%% achieved) and executed successfully",
    fraction * 100.0);
  return true;
}

bool TrajectoryPlanner::planAndExecuteInFrontOf(
  const StandoffConfig & config,
  rclcpp::Duration tf_timeout)
{
  const std::string & planning_frame = move_group_interface_.getPlanningFrame();

  geometry_msgs::msg::TransformStamped camera_tf;
  try {
    camera_tf = tf_buffer_.lookupTransform(
      planning_frame, config.camera_frame, tf2::TimePointZero,
      tf2::durationFromSec(tf_timeout.seconds()));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(
      node_->get_logger(), "Could not look up '%s' in planning frame '%s': %s",
      config.camera_frame.c_str(), planning_frame.c_str(), ex.what());
    return false;
  }

  const geometry_msgs::msg::Pose goal_pose =
    offsetInFrontOf(camera_tf, config.standoff_m, config.facing_rpy_rad);

  const double distance_from_origin = std::sqrt(
    goal_pose.position.x * goal_pose.position.x +
    goal_pose.position.y * goal_pose.position.y +
    goal_pose.position.z * goal_pose.position.z);

  if (distance_from_origin > config.max_reach_m) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Goal pose in front of '%s' is %.3fm from '%s', beyond max_reach_m (%.3fm) — "
      "refusing to plan. Lower standoff_m or check the TF.",
      config.camera_frame.c_str(), distance_from_origin, planning_frame.c_str(),
      config.max_reach_m);
    return false;
  }

  move_group_interface_.setEndEffectorLink(config.end_effector_frame);
  return planAndExecute(goal_pose);
}

bool TrajectoryPlanner::planAndExecuteInFrontOf(rclcpp::Duration tf_timeout)
{
  return planAndExecuteInFrontOf(standoff_config_, tf_timeout);
}

std::optional<std::pair<geometry_msgs::msg::Pose, bool>> TrajectoryPlanner::getStandoffPose(
  rclcpp::Duration tf_timeout) const
{
  const std::string & planning_frame = move_group_interface_.getPlanningFrame();

  geometry_msgs::msg::TransformStamped camera_tf;
  try {
    camera_tf = tf_buffer_.lookupTransform(
      planning_frame, standoff_config_.camera_frame, tf2::TimePointZero,
      tf2::durationFromSec(tf_timeout.seconds()));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Could not look up '%s' in planning frame '%s': %s — falling back to the "
      "'standoff' preset pose, if one is configured.",
      standoff_config_.camera_frame.c_str(), planning_frame.c_str(), ex.what());

    const std::optional<geometry_msgs::msg::Pose> preset = preset_poses_.get("standoff");
    if (!preset.has_value()) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "No 'standoff' preset configured either — cannot determine a standoff pose.");
      return std::nullopt;
    }
    return std::make_pair(*preset, true /*used_fallback*/);
  }

  const geometry_msgs::msg::Pose pose = offsetInFrontOf(
    camera_tf, standoff_config_.standoff_m, standoff_config_.facing_rpy_rad);
  return std::make_pair(pose, false /*used_fallback*/);
}

std::optional<geometry_msgs::msg::Pose> TrajectoryPlanner::getPresetPose(
  const std::string & name) const
{
  return preset_poses_.get(name);
}

bool TrajectoryPlanner::tracePath(
  const std::vector<geometry_msgs::msg::Pose> & waypoints,
  uint8_t planning_mode)
{
  move_group_interface_.setEndEffectorLink(standoff_config_.end_effector_frame);

  for (size_t i = 0; i < waypoints.size(); ++i) {
    RCLCPP_INFO(node_->get_logger(), "Tracing waypoint %zu/%zu", i + 1, waypoints.size());

    const bool succeeded =
      planning_mode == visual_calibration_msgs::srv::TracePath::Request::PLANNING_MODE_CARTESIAN ?
      planAndExecuteCartesian(waypoints[i]) :
      planAndExecute(waypoints[i]);

    if (!succeeded) {
      RCLCPP_ERROR(
        node_->get_logger(), "tracePath stopped at waypoint %zu/%zu", i + 1, waypoints.size());
      return false;
    }
  }
  return true;
}

std::vector<geometry_msgs::msg::Pose> TrajectoryPlanner::polygonWaypointsAroundStandoff(
  rclcpp::Duration tf_timeout) const
{
  if (polygon_config_.num_corners < 3) {
    RCLCPP_ERROR(
      node_->get_logger(), "polygon_num_corners (%d) must be >= 3", polygon_config_.num_corners);
    return {};
  }

  const std::string & planning_frame = move_group_interface_.getPlanningFrame();

  geometry_msgs::msg::TransformStamped camera_tf;
  try {
    camera_tf = tf_buffer_.lookupTransform(
      planning_frame, standoff_config_.camera_frame, tf2::TimePointZero,
      tf2::durationFromSec(tf_timeout.seconds()));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(
      node_->get_logger(), "Could not look up '%s' in planning frame '%s': %s",
      standoff_config_.camera_frame.c_str(), planning_frame.c_str(), ex.what());
    return {};
  }

  const geometry_msgs::msg::Pose standoff_pose = offsetInFrontOf(
    camera_tf, standoff_config_.standoff_m, standoff_config_.facing_rpy_rad);

  tf2::Transform standoff;
  tf2::fromMsg(standoff_pose, standoff);

  // Corner offsets applied in the standoff pose's own local X/Y plane (not
  // the camera's), so every corner keeps the same facing_rpy_rad-derived
  // orientation as the center — only position varies, so the target link
  // keeps facing the camera at each corner. Visited in angular order, so
  // consecutive waypoints are always adjacent (shorter individual moves).
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.reserve(static_cast<size_t>(polygon_config_.num_corners));
  for (int i = 0; i < polygon_config_.num_corners; ++i) {
    const double angle_rad = 2.0 * M_PI * static_cast<double>(i) / polygon_config_.num_corners;
    const tf2::Vector3 corner_offset(
      polygon_config_.radius_m * std::cos(angle_rad),
      polygon_config_.radius_m * std::sin(angle_rad),
      0.0);

    tf2::Transform corner(tf2::Quaternion::getIdentity(), corner_offset);
    tf2::Transform goal = standoff * corner;

    geometry_msgs::msg::Pose goal_pose;
    goal_pose.position.x = goal.getOrigin().x();
    goal_pose.position.y = goal.getOrigin().y();
    goal_pose.position.z = goal.getOrigin().z();
    goal_pose.orientation = tf2::toMsg(goal.getRotation());
    waypoints.push_back(goal_pose);
  }
  return waypoints;
}

std::vector<geometry_msgs::msg::Pose> TrajectoryPlanner::getPolygonWaypoints(
  rclcpp::Duration tf_timeout) const
{
  return polygonWaypointsAroundStandoff(tf_timeout);
}

void TrajectoryPlanner::handleTracePath(
  const std::shared_ptr<visual_calibration_msgs::srv::TracePath::Request> request,
  std::shared_ptr<visual_calibration_msgs::srv::TracePath::Response> response)
{
  const std::vector<geometry_msgs::msg::Pose> waypoints(
    request->waypoints.begin(), request->waypoints.end());

  response->success = tracePath(waypoints, request->planning_mode);
  response->message = response->success ?
    "Traced all waypoints successfully" : "Failed partway through tracing waypoints";

  // Only single-waypoint, named calls correspond to a meaningful "current
  // pose"/failure context for a human to see — see TracePath.srv's
  // pose_name doc comment.
  if (!request->pose_name.empty() && waypoints.size() == 1) {
    if (response->success) {
      publishCurrentPoseName(request->pose_name);
    } else {
      publishPlanningFailure(
        request->pose_name, "Move to '" + request->pose_name + "' failed: " + response->message);
    }
  }

  // See TracePath.srv's is_sequenced_goal doc comment — only single-waypoint
  // calls that opted in, and only once the move actually succeeded, start
  // the stay/lift/standby dance. waypoints[0] (the pose just reached) is
  // what onSequencedGoalReached computes the lift pose from — this design
  // does not track/return to any pose the arm was at before the goal.
  if (response->success && request->is_sequenced_goal && waypoints.size() == 1) {
    onSequencedGoalReached(waypoints[0]);
  }
}

void TrajectoryPlanner::handleTracePolygon(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  const std::vector<geometry_msgs::msg::Pose> waypoints =
    polygonWaypointsAroundStandoff(rclcpp::Duration::from_seconds(3.0));

  if (waypoints.empty()) {
    response->success = false;
    response->message = "Could not compute polygon waypoints (see log for the error)";
    return;
  }

  response->success = tracePath(waypoints, polygon_config_.default_planning_mode);
  response->message = response->success ?
    "Traced polygon path successfully" : "Failed partway through tracing polygon path";
}

void TrajectoryPlanner::handleGetPolygonWaypoints(
  const std::shared_ptr<visual_calibration_msgs::srv::GetPolygonWaypoints::Request>/*request*/,
  std::shared_ptr<visual_calibration_msgs::srv::GetPolygonWaypoints::Response> response)
{
  const std::vector<geometry_msgs::msg::Pose> waypoints =
    getPolygonWaypoints(rclcpp::Duration::from_seconds(3.0));

  response->success = !waypoints.empty();
  response->message = response->success ?
    "Computed polygon waypoints successfully" :
    "Could not compute polygon waypoints (see log for the error)";
  response->waypoints = std::vector<geometry_msgs::msg::Pose>(
    waypoints.begin(), waypoints.end());
}

void TrajectoryPlanner::handleGetStandoffPose(
  const std::shared_ptr<visual_calibration_msgs::srv::GetStandoffPose::Request>/*request*/,
  std::shared_ptr<visual_calibration_msgs::srv::GetStandoffPose::Response> response)
{
  const auto result = getStandoffPose(rclcpp::Duration::from_seconds(3.0));

  response->success = result.has_value();
  response->used_fallback = result.has_value() && result->second;
  if (!result.has_value()) {
    response->message = "Could not compute standoff pose via TF or preset fallback "
      "(see log for the error)";
    return;
  }

  response->standoff_pose = result->first;
  response->message = response->used_fallback ?
    "Camera TF unavailable — returned the 'standoff' preset pose instead" :
    "Computed standoff pose successfully via TF";
}

void TrajectoryPlanner::handleGetPresetPose(
  const std::shared_ptr<visual_calibration_msgs::srv::GetPresetPose::Request> request,
  std::shared_ptr<visual_calibration_msgs::srv::GetPresetPose::Response> response)
{
  const std::optional<geometry_msgs::msg::Pose> pose = getPresetPose(request->name);

  response->success = pose.has_value();
  response->message = response->success ?
    "Found preset pose '" + request->name + "'" :
    "No preset pose named '" + request->name + "' is configured";
  if (pose.has_value()) {
    response->pose = *pose;
  }
}

StandoffConfig TrajectoryPlanner::loadStandoffConfigFromParams() const
{
  StandoffConfig config;
  config.camera_frame = node_->get_parameter("camera_frame").as_string();
  config.end_effector_frame = node_->get_parameter("end_effector_frame").as_string();
  config.standoff_m = node_->get_parameter("standoff_m").as_double();
  config.max_reach_m = node_->get_parameter("max_reach_m").as_double();

  const std::vector<double> facing_rpy_rad =
    node_->get_parameter("facing_rpy_rad").as_double_array();
  config.facing_rpy_rad = {facing_rpy_rad[0], facing_rpy_rad[1], facing_rpy_rad[2]};

  return config;
}

PolygonConfig TrajectoryPlanner::loadPolygonConfigFromParams() const
{
  PolygonConfig config;
  config.num_corners = static_cast<int>(node_->get_parameter("polygon_num_corners").as_int());
  config.radius_m = node_->get_parameter("polygon_radius_m").as_double();

  const std::string mode_name = node_->get_parameter("polygon_default_planning_mode").as_string();
  if (mode_name == "cartesian") {
    config.default_planning_mode =
      visual_calibration_msgs::srv::TracePath::Request::PLANNING_MODE_CARTESIAN;
  } else if (mode_name == "joint_space") {
    config.default_planning_mode =
      visual_calibration_msgs::srv::TracePath::Request::PLANNING_MODE_JOINT_SPACE;
  } else {
    throw std::invalid_argument(
            "Unknown polygon_default_planning_mode: '" + mode_name +
            "' (expected 'cartesian' or 'joint_space')");
  }

  return config;
}

SequenceConfig TrajectoryPlanner::loadSequenceConfigFromParams() const
{
  SequenceConfig config;
  config.stay_seconds_at_goal =
    node_->get_parameter("stay_seconds_at_goal").as_double();
  config.lift_target_z_m =
    node_->get_parameter("lift_target_z_m").as_double();
  config.lift_wait_seconds =
    node_->get_parameter("lift_wait_seconds").as_double();
  return config;
}

void TrajectoryPlanner::publishCurrentPoseName(const std::string & name)
{
  std_msgs::msg::String msg;
  msg.data = name;
  current_pose_name_pub_->publish(msg);
}

void TrajectoryPlanner::publishPlanningFailure(
  const std::string & context, const std::string & message)
{
  visual_calibration_msgs::msg::PlanningFailure msg;
  msg.context = context;
  msg.message = message;
  planning_failure_pub_->publish(msg);
}

void TrajectoryPlanner::onSequencedGoalReached(const geometry_msgs::msg::Pose & goal_pose)
{
  // Held across the whole function, including timer creation — under
  // MultiThreadedExecutor a second sequenced goal could otherwise race in
  // between the cancel() calls below and the new timer being armed.
  std::lock_guard<std::mutex> lock(state_mutex_);

  // A new sequenced goal arriving mid-dance (e.g. while already
  // SETTLED_AT_GOAL or LIFTED_IDLE) cancels whatever was pending and
  // restarts fresh from here — this cancel is also the "idle" signal for
  // lift_wait_timer_ (see its member doc comment).
  if (stay_timer_) {
    stay_timer_->cancel();
  }
  if (lift_wait_timer_) {
    lift_wait_timer_->cancel();
  }

  goal_pose_ = goal_pose;
  arm_state_ = ArmState::SETTLED_AT_GOAL;

  stay_timer_ = node_->create_wall_timer(
    std::chrono::duration<double>(sequence_config_.stay_seconds_at_goal),
    [this]() {
      stay_timer_->cancel();  // one-shot: this timer's only job is done
      onStayTimerFired();
    });
}

void TrajectoryPlanner::onStayTimerFired()
{
  RCLCPP_INFO(
    node_->get_logger(),
    "Stayed at sequenced goal for %.1fs — lifting to Z=%.3fm.",
    sequence_config_.stay_seconds_at_goal, sequence_config_.lift_target_z_m);

  // Held across planAndExecute() deliberately — a concurrent sequenced
  // goal arriving mid-lift must wait for this move to finish (or be
  // cleanly superseded afterward), not race the arm's motion commands.
  std::lock_guard<std::mutex> lock(state_mutex_);

  // Absolute Z target in the planning frame (base_link) — NOT an offset
  // added to the goal's Z. X/Y/orientation stay identical to the goal —
  // see SequenceConfig::lift_target_z_m.
  geometry_msgs::msg::Pose lift_pose = goal_pose_;
  lift_pose.position.z = sequence_config_.lift_target_z_m;

  // See runStartupSequence's matching comment — preset/goal poses are
  // captured against end_effector_frame, must match it before planning.
  move_group_interface_.setEndEffectorLink(standoff_config_.end_effector_frame);
  const bool succeeded = planAndExecute(lift_pose);
  if (!succeeded) {
    const std::string message = "Failed to lift away from the sequenced goal.";
    RCLCPP_ERROR(node_->get_logger(), "%s", message.c_str());
    publishPlanningFailure("lift", message);
    arm_state_ = ArmState::IDLE;
    return;
  }

  arm_state_ = ArmState::LIFTED_IDLE;
  lift_wait_timer_ = node_->create_wall_timer(
    std::chrono::duration<double>(sequence_config_.lift_wait_seconds),
    [this]() {
      lift_wait_timer_->cancel();  // one-shot: this timer's only job is done
      onLiftWaitTimerFired();
    });
}

void TrajectoryPlanner::onLiftWaitTimerFired()
{
  RCLCPP_INFO(
    node_->get_logger(),
    "Idle for %.1fs with no new sequenced goal — moving to 'standby'.",
    sequence_config_.lift_wait_seconds);

  // Held across the lookup + planAndExecute() — same reasoning as
  // onStayTimerFired.
  std::lock_guard<std::mutex> lock(state_mutex_);

  const std::optional<geometry_msgs::msg::Pose> standby_pose = preset_poses_.get("standby");
  if (!standby_pose.has_value()) {
    const std::string message =
      "Idle timeout reached but no 'standby' preset is configured — staying at the "
      "current pose. See preset_poses_sim.yaml/_real.yaml.";
    RCLCPP_ERROR(node_->get_logger(), "%s", message.c_str());
    publishPlanningFailure("standby", message);
    arm_state_ = ArmState::IDLE;
    return;
  }

  // See runStartupSequence's matching comment — preset poses are captured
  // against end_effector_frame, must match it before planning.
  move_group_interface_.setEndEffectorLink(standoff_config_.end_effector_frame);
  const bool succeeded = planAndExecute(*standby_pose);
  if (!succeeded) {
    const std::string message = "Move to 'standby' pose failed.";
    RCLCPP_ERROR(node_->get_logger(), "%s", message.c_str());
    publishPlanningFailure("standby", message);
    arm_state_ = ArmState::IDLE;
    return;
  }

  arm_state_ = ArmState::STANDBY;
  publishCurrentPoseName("standby");
}

}  // namespace visual_calibration_moveit