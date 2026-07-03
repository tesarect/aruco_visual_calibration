#include "visual_calibration_moveit/simple_trajectory.hpp"

namespace visual_calibration_moveit
{

SimpleTrajectory::SimpleTrajectory()
: rclcpp::Node("simple_trajectory")
{
  RCLCPP_INFO(get_logger(), "simple_trajectory node started");
}

}  // namespace visual_calibration_moveit