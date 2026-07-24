#ifndef VISUAL_CALIBRATION_MOVEIT__PRESET_POSES_HPP_
#define VISUAL_CALIBRATION_MOVEIT__PRESET_POSES_HPP_

#include <map>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>

namespace visual_calibration_moveit
{

/// Named fallback poses for end_effector_frame, in the planning frame —
/// used when a deterministic pose can't be computed via TF (e.g. no
/// camera_frame TF yet on the real robot before first calibration — see
/// todo.txt item 1), OR to pin a move to one specific, already-verified
/// joint configuration rather than whatever IK solution the planner
/// happens to land on for a given Cartesian target (see cal_ready below).
/// Loaded from preset_poses_sim.yaml/_real.yaml, under a "preset_poses"
/// node with a preset_names string array and, per entry, EITHER a
/// "<name>.position"/"<name>.orientation" pair (Cartesian) OR a
/// "<name>.joint_values" 6-element array (joint-space) — open-ended by
/// design (unlike SceneObjectConfig's fixed SceneObjectId enum), since new
/// presets are just YAML additions with no corresponding code change.
///
/// Why a preset can need joint values instead of a pose: the UR3e has
/// multiple valid IK solutions (elbow-up/elbow-down, wrist-flipped, ...)
/// for the same Cartesian target — which one a joint-space plan lands in
/// depends on the path taken to get there, not just the target itself.
/// Confirmed 2026-07-20: two different joint-space paths to the exact same
/// cal_ready Cartesian pose (xyz/quat matching to 3+ decimal places)
/// produced joint configurations differing by 90-250° on several joints —
/// one of those configurations reliably let downstream Cartesian polygon-
/// corner moves succeed, the other reliably made them fail partway
/// (computeCartesianPath() stopping at ~80-83%). A Cartesian preset alone
/// can't force a specific IK branch; a joint-value preset can, via
/// MoveGroupInterface::setJointValueTarget() instead of setPoseTarget().
class PresetPoses
{
public:
  /// Reads preset_names and, per entry, either "<name>.position"/
  /// "<name>.orientation" or "<name>.joint_values" from node's declared
  /// parameters. A node started without a preset_poses config file (e.g.
  /// via automatically_declare_parameters_from_overrides with none
  /// provided) ends up with an empty preset set, not an error — every
  /// named preset is optional by design.
  explicit PresetPoses(const rclcpp::Node::SharedPtr & node);

  /// Returns the named preset's Cartesian pose, or std::nullopt if no
  /// Cartesian preset with that name was loaded (including if that name
  /// only has a joint-value preset — see getJointValues).
  std::optional<geometry_msgs::msg::Pose> get(const std::string & name) const;

  /// Returns the named preset's joint values (in the planning group's own
  /// joint order — see PresetPoses.cpp's loading comment), or std::nullopt
  /// if no joint-value preset with that name was loaded.
  std::optional<std::vector<double>> getJointValues(const std::string & name) const;

private:
  std::map<std::string, geometry_msgs::msg::Pose> poses_;
  std::map<std::string, std::vector<double>> joint_values_;
};

}  // namespace visual_calibration_moveit

#endif  // VISUAL_CALIBRATION_MOVEIT__PRESET_POSES_HPP_