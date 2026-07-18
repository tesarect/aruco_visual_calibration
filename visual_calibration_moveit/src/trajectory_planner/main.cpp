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

  // MultiThreadedExecutor, not SingleThreadedExecutor: TrajectoryPlanner's
  // handleTracePath calls move_group_interface_.getCurrentPose(), which
  // internally needs a /joint_states (via TF) callback to be serviced to
  // resolve — under a single-threaded executor, that callback can only run
  // on the same thread already blocked inside handleTracePath, deadlocking
  // the whole node (confirmed via testing, 2026-07-17: a plain named
  // ~/trace_path call hung indefinitely, no response, "No state update
  // received" repeating in the log). A multi-threaded executor lets a
  // second thread service that callback while the first is still blocked.
  // See TrajectoryPlanner's mutex-guarded state members (arm_state_ etc.)
  // for the thread-safety implication of this change — service/timer
  // callbacks can now genuinely run concurrently.
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);
  std::thread executor_thread([&executor]() {executor->spin();});

  // Never auto-moves to the standoff pose on startup — that still only
  // happens on explicit request (~/trace_path, ~/trace_polygon). The one
  // exception is an opt-in move to the "home" preset, gated behind the
  // move_to_home_on_startup param (see TrajectoryPlanner::runStartupSequence,
  // called from the constructor) — an explicit, auditable config choice
  // rather than unconditional motion; set per-environment in
  // trajectory_planner_sim.yaml/_real.yaml (both true today, matching the
  // desired "always return home on startup" behavior — flip to false there
  // if this should be paused without a code change). See todo.txt item 1
  // and error-mitigation.md for why
  // unconditional motion on startup is unsafe on the real robot (no TF
  // guarantee, no operator confirmation) — that reasoning is exactly why
  // this is opt-in rather than the default behavior.
  visual_calibration_moveit::TrajectoryPlanner trajectory_planner(node);

  // Keep the node alive so ~/trace_path, ~/trace_polygon,
  // ~/get_polygon_waypoints, and ~/get_standoff_pose (see
  // TrajectoryPlanner's constructor) stay reachable for the node's full
  // lifetime — these are services, which need a live node to be callable.
  executor_thread.join();
  rclcpp::shutdown();
  return 0;
}