#include <rclcpp/rclcpp.hpp>

#include "orchestrator/calibration_orchestrator_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // MultiThreadedExecutor (not plain rclcpp::spin's default
  // SingleThreadedExecutor) — needed so a worker thread remains free to
  // service AsyncParametersClient responses (see
  // classical_detector_param_client_'s doc comment in the header) while
  // another thread is inside a long-running callback like
  // centerOnMarkerUsingImage that polls one of those futures. With a
  // single-threaded executor that poll would deadlock: the one thread
  // doing the polling is the same thread that would need to process the
  // incoming response.
  auto node = std::make_shared<orchestrator::CalibrationOrchestratorNode>();
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);
  executor->spin();
  rclcpp::shutdown();
  return 0;
}