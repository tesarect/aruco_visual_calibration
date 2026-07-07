#include "visual_calibration_moveit/trajectory_planner.hpp"

#include <cmath>
#include <vector>

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
  standoff_config_(loadStandoffConfigFromParams())
{
  RCLCPP_INFO(
    node_->get_logger(), "trajectory_planner ready (planning group: '%s')",
    planning_group.c_str());
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

}  // namespace visual_calibration_moveit