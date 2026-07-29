#include <rclcpp/rclcpp.hpp>

#include "aruco_perception/cup_holder_detector_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<aruco_perception::CupHolderDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
