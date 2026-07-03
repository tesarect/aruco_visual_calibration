#ifndef VISUAL_CALIBRATION_MOVEIT__MTC_TRAJECTORY_HPP_
#define VISUAL_CALIBRATION_MOVEIT__MTC_TRAJECTORY_HPP_

#include <rclcpp/rclcpp.hpp>

namespace visual_calibration_moveit
{

/// Staged trajectory generation via MoveIt Task Constructor: chains
/// approach / interact / retreat (and gripper open/close) as distinct
/// stages instead of one plan+execute call. See SimpleTrajectory for
/// the single-pose alternative.
class MtcTrajectory : public rclcpp::Node
{
public:
  MtcTrajectory();

private:
  // TODO: build the MTC Task (stages), plan, execute.
};

}  // namespace visual_calibration_moveit

#endif  // VISUAL_CALIBRATION_MOVEIT__MTC_TRAJECTORY_HPP_