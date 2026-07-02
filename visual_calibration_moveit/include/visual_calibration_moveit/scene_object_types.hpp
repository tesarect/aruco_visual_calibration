#ifndef VISUAL_CALIBRATION_MOVEIT__SCENE_OBJECT_TYPES_HPP_
#define VISUAL_CALIBRATION_MOVEIT__SCENE_OBJECT_TYPES_HPP_

#include <string>

namespace visual_calibration_moveit
{

enum class SceneObjectId
{
  CoffeeMachine,
  ArmPlatform,
};

/// Param namespace each SceneObjectId is declared/read under, e.g.
/// "coffee_machine.pose.x", "coffee_machine.mesh_path".
std::string toParamPrefix(SceneObjectId id);

/// CollisionObject id/name published into the planning scene.
std::string toObjectName(SceneObjectId id);

struct Pose2D
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0;
};

struct SceneObjectConfig
{
  SceneObjectId id;
  std::string mesh_path;
  Pose2D pose;
};

}  // namespace visual_calibration_moveit

#endif  // VISUAL_CALIBRATION_MOVEIT__SCENE_OBJECT_TYPES_HPP_