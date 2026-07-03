#include "visual_calibration_moveit/mtc_trajectory.hpp"

namespace visual_calibration_moveit
{

MtcTrajectory::MtcTrajectory()
: rclcpp::Node("mtc_trajectory")
{
  RCLCPP_INFO(get_logger(), "mtc_trajectory node started");
}

}  // namespace visual_calibration_moveit