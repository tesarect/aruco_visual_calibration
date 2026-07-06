#include <rclcpp/rclcpp.hpp>

#include "aruco_perception/image_subscriber_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<aruco_perception::ImageSubscriberNode>());
  rclcpp::shutdown();
  return 0;
}