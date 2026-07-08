#include <rclcpp/rclcpp.hpp>

#include "aruco_perception/calibration_broadcaster_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<aruco_perception::CalibrationBroadcasterNode>());
  rclcpp::shutdown();
  return 0;
}
