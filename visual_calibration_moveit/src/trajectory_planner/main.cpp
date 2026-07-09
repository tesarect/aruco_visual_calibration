#include <rclcpp/rclcpp.hpp>

#include "visual_calibration_moveit/trajectory_planner.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // automatically_declare_parameters_from_overrides is required by
  // MoveGroupInterface, matching the official MoveIt2 tutorial pattern.
  auto node = std::make_shared<rclcpp::Node>(
    "trajectory_planner",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
  std::thread executor_thread([&executor]() {executor->spin();});

  visual_calibration_moveit::TrajectoryPlanner trajectory_planner(node);
  trajectory_planner.planAndExecuteInFrontOf();

  // Keep the node alive after the initial move: ~/trace_path and
  // ~/trace_polygon (see TrajectoryPlanner's constructor) are services,
  // which need a live node to be callable at all. Exiting here right
  // after the first move (the previous behavior) meant those services
  // were never actually reachable.
  executor_thread.join();
  rclcpp::shutdown();
  return 0;
}