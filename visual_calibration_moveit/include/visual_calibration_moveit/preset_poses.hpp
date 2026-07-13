#ifndef VISUAL_CALIBRATION_MOVEIT__PRESET_POSES_HPP_
#define VISUAL_CALIBRATION_MOVEIT__PRESET_POSES_HPP_

#include <map>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>

namespace visual_calibration_moveit
{

/// Named fallback Cartesian poses for end_effector_frame, in the planning
/// frame — used when a deterministic pose can't be computed via TF (e.g.
/// no camera_frame TF yet on the real robot before first calibration —
/// see todo.txt item 1). Loaded from preset_poses_sim.yaml/_real.yaml,
/// under a "preset_poses" node with a preset_names string array and one
/// "<name>.position"/"<name>.orientation" pair per entry — open-ended by
/// design (unlike SceneObjectConfig's fixed SceneObjectId enum), since new
/// presets are just YAML additions with no corresponding code change.
class PresetPoses
{
public:
  /// Reads preset_names and each "<name>.position"/"<name>.orientation"
  /// pair from node's declared parameters. A node started without a
  /// preset_poses config file (e.g. via automatically_declare_parameters_
  /// from_overrides with none provided) ends up with an empty preset set,
  /// not an error — every named preset is optional by design.
  explicit PresetPoses(const rclcpp::Node::SharedPtr & node);

  /// Returns the named preset's pose, or std::nullopt if no preset with
  /// that name was loaded.
  std::optional<geometry_msgs::msg::Pose> get(const std::string & name) const;

private:
  std::map<std::string, geometry_msgs::msg::Pose> poses_;
};

}  // namespace visual_calibration_moveit

#endif  // VISUAL_CALIBRATION_MOVEIT__PRESET_POSES_HPP_