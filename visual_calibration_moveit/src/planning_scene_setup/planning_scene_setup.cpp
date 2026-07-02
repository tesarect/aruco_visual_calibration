#include "visual_calibration_moveit/planning_scene_setup.hpp"

#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <shape_msgs/msg/mesh.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace visual_calibration_moveit
{

PlanningSceneSetup::PlanningSceneSetup()
: rclcpp::Node("planning_scene_setup")
{
  declareParameters();
  planning_frame_ = get_parameter("planning_frame").as_string();

  addCoffeeMachine();
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
}

moveit_msgs::msg::CollisionObject PlanningSceneSetup::makeMeshCollisionObject(
  const std::string & object_id,
  const std::string & param_prefix)
{
  double x = get_parameter(param_prefix + ".pose.x").as_double();
  double y = get_parameter(param_prefix + ".pose.y").as_double();
  double z = get_parameter(param_prefix + ".pose.z").as_double();
  double yaw = get_parameter(param_prefix + ".pose.yaw").as_double();
  std::string mesh_path = get_parameter(param_prefix + ".mesh_path").as_string();

  shapes::Mesh * mesh = shapes::createMeshFromResource(mesh_path);
  shapes::ShapeMsg shape_msg;
  shapes::constructMsgFromShape(mesh, shape_msg);
  delete mesh;

  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = planning_frame_;
  object.id = object_id;
  object.meshes.push_back(boost::get<shape_msgs::msg::Mesh>(shape_msg));

  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  object.mesh_poses.push_back(pose);

  object.operation = object.ADD;
  return object;
}

void PlanningSceneSetup::addCoffeeMachine()
{
  auto object = makeMeshCollisionObject("coffee_machine", "coffee_machine");
  planning_scene_interface_.applyCollisionObject(object);
  RCLCPP_INFO(get_logger(), "Added 'coffee_machine' to the planning scene");
}

}  // namespace visual_calibration_moveit
