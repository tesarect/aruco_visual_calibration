#include <rclcpp/rclcpp.hpp>

#include "orchestrator/calibration_orchestrator_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<orchestrator::CalibrationOrchestratorNode>());
  rclcpp::shutdown();
  return 0;
}