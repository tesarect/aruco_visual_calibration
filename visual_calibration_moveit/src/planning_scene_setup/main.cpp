#include <rclcpp/rclcpp.hpp>

#include "visual_calibration_moveit/planning_scene_setup.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<visual_calibration_moveit::PlanningSceneSetup>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}