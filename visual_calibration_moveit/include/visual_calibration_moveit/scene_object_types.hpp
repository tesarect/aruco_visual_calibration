#ifndef VISUAL_CALIBRATION_MOVEIT__SCENE_OBJECT_TYPES_HPP_
#define VISUAL_CALIBRATION_MOVEIT__SCENE_OBJECT_TYPES_HPP_

#include <string>
#include <vector>

namespace visual_calibration_moveit
{

enum class SceneObjectId
{
  CoffeeMachine,
  Cupholder,
  Countertop,
};

/// How a SceneObjectConfig's geometry should be interpreted:
/// - Mesh: load `mesh_path` (a .dae/.stl resource) as the collision shape.
/// - Box: use `boxes` — one or more axis-aligned boxes, each with its own
///   pose local to the object's base `pose` (e.g. the countertop's body +
///   thin top slab, matching how it's actually modeled in the Gazebo SDF).
enum class ShapeType
{
  Mesh,
  Box,
};

struct Pose2D
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0;
};

struct BoxShape
{
  double size_x = 0.0;
  double size_y = 0.0;
  double size_z = 0.0;
  Pose2D local_pose;  // offset from the object's base pose
};

struct SceneObjectConfig
{
  SceneObjectId id;
  ShapeType shape_type = ShapeType::Mesh;
  Pose2D pose;  // object's base pose in the planning frame

  std::string mesh_path;       // used when shape_type == ShapeType::Mesh
  std::vector<BoxShape> boxes; // used when shape_type == ShapeType::Box
};

/// Param namespace each SceneObjectId is declared/read under, e.g.
/// "coffee_machine.pose.x", "coffee_machine.mesh_path".
std::string toParamPrefix(SceneObjectId id);

/// CollisionObject id/name published into the planning scene.
std::string toObjectName(SceneObjectId id);

}  // namespace visual_calibration_moveit

#endif  // VISUAL_CALIBRATION_MOVEIT__SCENE_OBJECT_TYPES_HPP_
