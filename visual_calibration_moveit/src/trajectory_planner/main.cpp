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

  // Deliberately does NOT auto-move to the standoff pose on startup — the
  // node only executes motion when explicitly asked (~/trace_path,
  // ~/trace_polygon), matching its role as a plain executor with no
  // opinion on when the arm should move. A caller that wants the standoff
  // pose specifically should call ~/get_standoff_pose first (read-only,
  // no motion) to check whether a deterministic pose is available via TF,
  // then ~/trace_path with that pose (or a preset-pose fallback) to
  // actually move — see todo.txt item 1 and error-mitigation.md for why
  // unconditional motion on startup is unsafe on the real robot (no TF
  // guarantee, no operator confirmation).
  visual_calibration_moveit::TrajectoryPlanner trajectory_planner(node);

  // Keep the node alive so ~/trace_path, ~/trace_polygon,
  // ~/get_polygon_waypoints, and ~/get_standoff_pose (see
  // TrajectoryPlanner's constructor) stay reachable for the node's full
  // lifetime — these are services, which need a live node to be callable.
  executor_thread.join();
  rclcpp::shutdown();
  return 0;
}