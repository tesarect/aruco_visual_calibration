#include <rclcpp/rclcpp.hpp>

#include "visual_calibration_moveit/simple_trajectory.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // automatically_declare_parameters_from_overrides is required by
  // MoveGroupInterface, matching the official MoveIt2 tutorial pattern.
  auto node = std::make_shared<rclcpp::Node>(
    "simple_trajectory",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
  std::thread executor_thread([&executor]() {executor->spin();});

  visual_calibration_moveit::SimpleTrajectory simple_trajectory(node);
  simple_trajectory.planAndExecuteInFrontOf();

  rclcpp::shutdown();
  executor_thread.join();
  return 0;
}