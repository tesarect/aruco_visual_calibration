#ifndef VISUAL_CALIBRATION_MOVEIT__PLANNING_SCENE_SETUP_HPP_
#define VISUAL_CALIBRATION_MOVEIT__PLANNING_SCENE_SETUP_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>

namespace visual_calibration_moveit
{

/// Populates the MoveIt2 planning scene with known static obstacles
/// (coffee machine, arm platform) so trajectory planning avoids them.
class PlanningSceneSetup : public rclcpp::Node
{
public:
  PlanningSceneSetup();

private:
  void declareParameters();
  void addCoffeeMachine();

  moveit_msgs::msg::CollisionObject makeMeshCollisionObject(
    const std::string & object_id,
    const std::string & param_prefix);

  std::string planning_frame_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
};

}  // namespace visual_calibration_moveit

#endif  // VISUAL_CALIBRATION_MOVEIT__PLANNING_SCENE_SETUP_HPP_