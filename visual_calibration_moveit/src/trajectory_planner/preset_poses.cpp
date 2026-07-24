#include "visual_calibration_moveit/preset_poses.hpp"

namespace visual_calibration_moveit
{

namespace
{
/// A YAML `preset_names: []` (empty list) has no element type ROS 2 can
/// infer, and crashes the node at startup (InvalidParameterValueException)
/// before any node code runs — see preset_poses_sim.yaml's comment. Files
/// with no real presets use this single-element sentinel array instead,
/// which declares cleanly (unambiguous string type) — skip it here rather
/// than treating it as a real preset name.
constexpr const char * kNoPresetsSentinel = "__none__";
}  // namespace

PresetPoses::PresetPoses(const rclcpp::Node::SharedPtr & node)
{
  if (!node->has_parameter("preset_names")) {
    return;
  }

  const std::vector<std::string> preset_names =
    node->get_parameter("preset_names").as_string_array();

  for (const std::string & name : preset_names) {
    if (name == kNoPresetsSentinel) {
      continue;
    }

    // joint_values takes priority over position/orientation if a preset
    // somehow declares both (shouldn't happen in practice — each preset
    // is meant to be one or the other, see preset_poses.hpp's doc
    // comment) — checked first since a 6-element joint_values array is
    // the more specific/intentional choice when present.
    if (node->has_parameter(name + ".joint_values")) {
      const std::vector<double> joint_values =
        node->get_parameter(name + ".joint_values").as_double_array();

      if (joint_values.size() != 6) {
        RCLCPP_ERROR(
          node->get_logger(),
          "Preset '%s' has a malformed joint_values (expected 6 elements, "
          "got %zu) — skipping this preset.",
          name.c_str(), joint_values.size());
        continue;
      }

      joint_values_[name] = joint_values;
      continue;
    }

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

std::optional<std::vector<double>> PresetPoses::getJointValues(const std::string & name) const
{
  const auto it = joint_values_.find(name);
  if (it == joint_values_.end()) {
    return std::nullopt;
  }
  return it->second;
}

}  // namespace visual_calibration_moveit