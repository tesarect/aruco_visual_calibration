#ifndef ARUCO_PERCEPTION__ORIENTATION_AVERAGING_HPP_
#define ARUCO_PERCEPTION__ORIENTATION_AVERAGING_HPP_

#include <vector>

#include <tf2/LinearMath/Quaternion.h>

namespace aruco_perception
{

/// Orientation-averaging strategies for CalibrationBroadcasterNode. Chosen
/// by priority (see selectAveragingMethod), not directly by name, so
/// additional methods can be chained/escalated to later without changing
/// the selection interface.
enum class OrientationAveragingMethod
{
  /// Sum all sample quaternions component-wise, then renormalize to unit
  /// length. Correct as long as samples are reasonably close together
  /// (true here — same physical marker/camera, only per-frame noise
  /// differs) — not a proper SO(3) average for widely-spread samples.
  kSumNormalize,
  /// Markley's eigenvalue method (proper SO(3) average, robust to
  /// widely-spread samples) — NOT YET IMPLEMENTED. Reserved so priority
  /// configs can name it now; averageQuaternions throws
  /// std::invalid_argument if this is actually selected.
  kMarkley,
};

/// Result of averaging N orientation samples: the averaged quaternion plus
/// how far each sample deviated from it (angular spread, in degrees) — a
/// quality signal for whether the average is trustworthy, independent of
/// which method produced it. Not yet used to auto-escalate between
/// methods (see progress.md's Feature Additions) — logged for now.
struct OrientationAveragingResult
{
  tf2::Quaternion averaged;
  double max_spread_deg = 0.0;
  double mean_spread_deg = 0.0;
};

/// Picks the highest-priority (lowest positive priority number) method
/// among those given, in kSumNormalize/kMarkley order for ties. A priority
/// of 0 means "disabled" — that method is never selected. Throws
/// std::invalid_argument if every priority is 0 (no method enabled).
OrientationAveragingMethod selectAveragingMethod(
  int sum_normalize_priority, int markley_priority);

/// Averages `samples` (must be non-empty) using `method`. Throws
/// std::invalid_argument if method is kMarkley (not yet implemented) or if
/// samples is empty.
OrientationAveragingResult averageQuaternions(
  const std::vector<tf2::Quaternion> & samples,
  OrientationAveragingMethod method);

}  // namespace aruco_perception

#endif  // ARUCO_PERCEPTION__ORIENTATION_AVERAGING_HPP_