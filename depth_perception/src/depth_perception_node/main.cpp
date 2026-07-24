#include <rclcpp/rclcpp.hpp>

#include "depth_perception/depth_perception_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<depth_perception::DepthPerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
