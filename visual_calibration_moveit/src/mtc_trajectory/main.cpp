#include <rclcpp/rclcpp.hpp>

#include "visual_calibration_moveit/mtc_trajectory.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<visual_calibration_moveit::MtcTrajectory>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}