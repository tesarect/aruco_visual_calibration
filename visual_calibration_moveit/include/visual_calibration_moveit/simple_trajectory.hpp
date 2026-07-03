#ifndef VISUAL_CALIBRATION_MOVEIT__SIMPLE_TRAJECTORY_HPP_
#define VISUAL_CALIBRATION_MOVEIT__SIMPLE_TRAJECTORY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

namespace visual_calibration_moveit
{

/// One-shot trajectory generation via MoveGroupInterface: set a target
/// pose (e.g. derived from the camera->base_link TF chain), plan, execute.
/// No staged approach/retreat/gripper choreography — see MtcTrajectory
/// for that.
class SimpleTrajectory : public rclcpp::Node
{
public:
  SimpleTrajectory();

private:
  // TODO: target pose source (TF lookup), plan/execute call.
};

}  // namespace visual_calibration_moveit

#endif  // VISUAL_CALIBRATION_MOVEIT__SIMPLE_TRAJECTORY_HPP_