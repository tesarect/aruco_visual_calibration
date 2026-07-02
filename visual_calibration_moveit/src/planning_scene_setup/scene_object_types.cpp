#include "visual_calibration_moveit/scene_object_types.hpp"

#include <stdexcept>

namespace visual_calibration_moveit
{

std::string toParamPrefix(SceneObjectId id)
{
  switch (id) {
    case SceneObjectId::CoffeeMachine:
      return "coffee_machine";
    case SceneObjectId::Cupholder:
      return "cupholder";
    case SceneObjectId::Countertop:
      return "countertop";
  }
  throw std::invalid_argument("Unknown SceneObjectId");
}

std::string toObjectName(SceneObjectId id)
{
  // Currently identical to the param prefix; kept as a separate function
  // since the published CollisionObject id and the param namespace are
  // conceptually different things that happen to share a value today.
  return toParamPrefix(id);
}

}  // namespace visual_calibration_moveit
