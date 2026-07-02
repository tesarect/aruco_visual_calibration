#include "visual_calibration_moveit/planning_scene_setup.hpp"

#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <shape_msgs/msg/mesh.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace visual_calibration_moveit
{

const std::vector<SceneObjectId> PlanningSceneSetup::kKnownObjectIds = {
  SceneObjectId::CoffeeMachine,
  SceneObjectId::ArmPlatform,
};

PlanningSceneSetup::PlanningSceneSetup()
: rclcpp::Node("planning_scene_setup")
{
  declareParameters();
  planning_frame_ = get_parameter("planning_frame").as_string();

  addAllObjects();
}

void PlanningSceneSetup::declareParameters()
{
  declare_parameter("planning_frame", "base_link");

  declare_parameter("coffee_machine.pose.x", 0.1);
  declare_parameter("coffee_machine.pose.y", 0.86);
  declare_parameter("coffee_machine.pose.z", -0.032);
  declare_parameter("coffee_machine.pose.yaw", 1.57);
  declare_parameter(
    "coffee_machine.mesh_path",
    "package://the_construct_office_gazebo/models/coffee_machine/meshes/cafeteria.dae");

  declare_parameter("arm_platform.pose.x", -0.26);
  declare_parameter("arm_platform.pose.y", 0.04);
  declare_parameter("arm_platform.pose.z", -0.632);
  declare_parameter("arm_platform.pose.yaw", 1.57);
  declare_parameter(
    "arm_platform.mesh_path",
    "package://the_construct_office_gazebo/models/barista_model/meshes/TOP_colour_1.dae");
}

std::vector<SceneObjectConfig> PlanningSceneSetup::loadSceneObjects()
{
  std::vector<SceneObjectConfig> configs;
  configs.reserve(kKnownObjectIds.size());

  for (const auto & id : kKnownObjectIds) {
    const std::string prefix = toParamPrefix(id);

    SceneObjectConfig config;
    config.id = id;
    config.mesh_path = get_parameter(prefix + ".mesh_path").as_string();
    config.pose.x = get_parameter(prefix + ".pose.x").as_double();
    config.pose.y = get_parameter(prefix + ".pose.y").as_double();
    config.pose.z = get_parameter(prefix + ".pose.z").as_double();
    config.pose.yaw = get_parameter(prefix + ".pose.yaw").as_double();

    configs.push_back(config);
  }

  return configs;
}

moveit_msgs::msg::CollisionObject PlanningSceneSetup::toCollisionObject(
  const SceneObjectConfig & config)
{
  shapes::Mesh * mesh = shapes::createMeshFromResource(config.mesh_path);
  shapes::ShapeMsg shape_msg;
  shapes::constructMsgFromShape(mesh, shape_msg);
  delete mesh;

  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = planning_frame_;
  object.id = toObjectName(config.id);
  object.meshes.push_back(boost::get<shape_msgs::msg::Mesh>(shape_msg));

  geometry_msgs::msg::Pose pose;
  pose.position.x = config.pose.x;
  pose.position.y = config.pose.y;
  pose.position.z = config.pose.z;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, config.pose.yaw);
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  object.mesh_poses.push_back(pose);

  object.operation = object.ADD;
  return object;
}

void PlanningSceneSetup::addAllObjects()
{
  for (const auto & config : loadSceneObjects()) {
    auto object = toCollisionObject(config);
    planning_scene_interface_.applyCollisionObject(object);
    RCLCPP_INFO(get_logger(), "Added '%s' to the planning scene", object.id.c_str());
  }
}

}  // namespace visual_calibration_moveit