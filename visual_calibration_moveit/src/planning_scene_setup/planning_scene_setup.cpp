#include "visual_calibration_moveit/planning_scene_setup.hpp"

#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <shape_msgs/msg/mesh.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace visual_calibration_moveit
{

namespace
{

// File-local helper: gets internal linkage from the anonymous namespace
// since nothing outside this .cpp needs it and it isn't declared in the
// header — it's a private implementation detail, not part of the class's
// public surface.
geometry_msgs::msg::Pose toGeometryPose(const Pose2D & pose)
{
  geometry_msgs::msg::Pose result;
  result.position.x = pose.x;
  result.position.y = pose.y;
  result.position.z = pose.z;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, pose.yaw);
  result.orientation.x = q.x();
  result.orientation.y = q.y();
  result.orientation.z = q.z();
  result.orientation.w = q.w();
  return result;
}

}  // namespace

const std::vector<SceneObjectId> PlanningSceneSetup::kKnownObjectIds = {
  SceneObjectId::CoffeeMachine,
  SceneObjectId::Cupholder,
  SceneObjectId::Countertop,
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

  declare_parameter("coffee_machine.shape_type", "mesh");
  declare_parameter("coffee_machine.pose.x", 0.1);
  declare_parameter("coffee_machine.pose.y", 0.86);
  declare_parameter("coffee_machine.pose.z", -0.032);
  declare_parameter("coffee_machine.pose.yaw", 1.57);
  declare_parameter(
    "coffee_machine.mesh_path",
    "package://the_construct_office_gazebo/models/coffee_machine/meshes/cafeteria.dae");

  declare_parameter("cupholder.shape_type", "mesh");
  declare_parameter("cupholder.pose.x", -0.26);
  declare_parameter("cupholder.pose.y", 0.04);
  declare_parameter("cupholder.pose.z", -0.632);
  declare_parameter("cupholder.pose.yaw", 1.57);
  declare_parameter(
    "cupholder.mesh_path",
    "package://the_construct_office_gazebo/models/barista_model/meshes/TOP_colour_1.dae");

  // Countertop is modeled in the Gazebo SDF as two stacked box primitives
  // (body + a thinner top slab), not a mesh — see
  // the_construct_office_gazebo/models/starbots_bartender_dispenser/model.sdf
  declare_parameter("countertop.shape_type", "box");
  declare_parameter("countertop.pose.x", 0.3);
  declare_parameter("countertop.pose.y", 0.36);
  declare_parameter("countertop.pose.z", -0.532);
  declare_parameter("countertop.pose.yaw", 0.0);

  declare_parameter("countertop.boxes.body.size", std::vector<double>{0.5, 1.8, 1.0});
  declare_parameter("countertop.boxes.body.local_pose", std::vector<double>{0.0, 0.0, 0.0, 0.0});

  declare_parameter("countertop.boxes.top.size", std::vector<double>{0.85, 1.81, 0.05});
  declare_parameter("countertop.boxes.top.local_pose", std::vector<double>{0.0, 0.0, 0.5, 0.0});
}

std::vector<SceneObjectConfig> PlanningSceneSetup::loadSceneObjects()
{
  std::vector<SceneObjectConfig> configs;
  configs.reserve(kKnownObjectIds.size());

  for (const auto & id : kKnownObjectIds) {
    const std::string prefix = toParamPrefix(id);

    SceneObjectConfig config;
    config.id = id;
    config.pose.x = get_parameter(prefix + ".pose.x").as_double();
    config.pose.y = get_parameter(prefix + ".pose.y").as_double();
    config.pose.z = get_parameter(prefix + ".pose.z").as_double();
    config.pose.yaw = get_parameter(prefix + ".pose.yaw").as_double();

    const std::string shape_type = get_parameter(prefix + ".shape_type").as_string();

    if (shape_type == "box") {
      config.shape_type = ShapeType::Box;

      for (const char * box_name : {"body", "top"}) {
        const auto size = get_parameter(
          prefix + ".boxes." + box_name + ".size").as_double_array();
        const auto local_pose = get_parameter(
          prefix + ".boxes." + box_name + ".local_pose").as_double_array();

        BoxShape box;
        box.size_x = size.at(0);
        box.size_y = size.at(1);
        box.size_z = size.at(2);
        box.local_pose.x = local_pose.at(0);
        box.local_pose.y = local_pose.at(1);
        box.local_pose.z = local_pose.at(2);
        box.local_pose.yaw = local_pose.at(3);

        config.boxes.push_back(box);
      }
    } else {
      config.shape_type = ShapeType::Mesh;
      config.mesh_path = get_parameter(prefix + ".mesh_path").as_string();
    }

    configs.push_back(config);
  }

  return configs;
}

moveit_msgs::msg::CollisionObject PlanningSceneSetup::meshObjectToCollisionObject(
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
  object.mesh_poses.push_back(toGeometryPose(config.pose));
  object.operation = object.ADD;
  return object;
}

moveit_msgs::msg::CollisionObject PlanningSceneSetup::boxObjectToCollisionObject(
  const SceneObjectConfig & config)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = planning_frame_;
  object.id = toObjectName(config.id);

  for (const auto & box : config.boxes) {
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {box.size_x, box.size_y, box.size_z};
    object.primitives.push_back(primitive);

    // local_pose is relative to the object's base pose; combine them so
    // each box lands at its correct position in the planning frame.
    Pose2D combined;
    combined.x = config.pose.x + box.local_pose.x;
    combined.y = config.pose.y + box.local_pose.y;
    combined.z = config.pose.z + box.local_pose.z;
    combined.yaw = config.pose.yaw + box.local_pose.yaw;
    object.primitive_poses.push_back(toGeometryPose(combined));
  }

  object.operation = object.ADD;
  return object;
}

moveit_msgs::msg::CollisionObject PlanningSceneSetup::toCollisionObject(
  const SceneObjectConfig & config)
{
  switch (config.shape_type) {
    case ShapeType::Mesh:
      return meshObjectToCollisionObject(config);
    case ShapeType::Box:
      return boxObjectToCollisionObject(config);
  }
  throw std::invalid_argument("Unknown ShapeType");
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
