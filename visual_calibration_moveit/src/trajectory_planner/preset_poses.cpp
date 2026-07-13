#include "visual_calibration_moveit/preset_poses.hpp"

namespace visual_calibration_moveit
{

PresetPoses::PresetPoses(const rclcpp::Node::SharedPtr & node)
{
  if (!node->has_parameter("preset_names")) {
    return;
  }

  const std::vector<std::string> preset_names =
    node->get_parameter("preset_names").as_string_array();

  for (const std::string & name : preset_names) {
    const std::vector<double> position =
      node->get_parameter(name + ".position").as_double_array();
    const std::vector<double> orientation =
      node->get_parameter(name + ".orientation").as_double_array();

    if (position.size() != 3 || orientation.size() != 4) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Preset '%s' has a malformed position/orientation (expected 3/4 "
        "elements, got %zu/%zu) — skipping this preset.",
        name.c_str(), position.size(), orientation.size());
      continue;
    }

    geometry_msgs::msg::Pose pose;
    pose.position.x = position[0];
    pose.position.y = position[1];
    pose.position.z = position[2];
    pose.orientation.x = orientation[0];
    pose.orientation.y = orientation[1];
    pose.orientation.z = orientation[2];
    pose.orientation.w = orientation[3];

    poses_[name] = pose;
  }
}

std::optional<geometry_msgs::msg::Pose> PresetPoses::get(const std::string & name) const
{
  const auto it = poses_.find(name);
  if (it == poses_.end()) {
    return std::nullopt;
  }
  return it->second;
}

}  // namespace visual_calibration_moveit