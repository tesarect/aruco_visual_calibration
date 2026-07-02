#ifndef VISUAL_CALIBRATION_MOVEIT__PLANNING_SCENE_SETUP_HPP_
#define VISUAL_CALIBRATION_MOVEIT__PLANNING_SCENE_SETUP_HPP_

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>

#include "visual_calibration_moveit/scene_object_types.hpp"

namespace visual_calibration_moveit
{

/// Populates the MoveIt2 planning scene with known static obstacles
/// (coffee machine, cupholder, countertop, wall) so trajectory planning
/// avoids them.
class PlanningSceneSetup : public rclcpp::Node
{
public:
  PlanningSceneSetup();

private:
  void declareParameters();
  std::vector<SceneObjectConfig> loadSceneObjects();
  moveit_msgs::msg::CollisionObject toCollisionObject(const SceneObjectConfig & config);
  moveit_msgs::msg::CollisionObject meshObjectToCollisionObject(const SceneObjectConfig & config);
  moveit_msgs::msg::CollisionObject boxObjectToCollisionObject(const SceneObjectConfig & config);
  void addAllObjects();

  static const std::vector<SceneObjectId> kKnownObjectIds;

  std::string planning_frame_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
};

}  // namespace visual_calibration_moveit

#endif  // VISUAL_CALIBRATION_MOVEIT__PLANNING_SCENE_SETUP_HPP_
