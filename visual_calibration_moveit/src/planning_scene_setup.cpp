#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("planning_scene_setup");

  RCLCPP_INFO(node->get_logger(), "planning_scene_setup node started");

  rclcpp::shutdown();
  return 0;
}