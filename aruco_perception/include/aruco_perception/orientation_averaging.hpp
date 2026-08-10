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
  /// Markley's eigenvalue method (Markley, Cheng, Crassidis, Oshman,
  /// "Averaging Quaternions," JGCD 2007) — the proper SO(3) average,
  /// robust to widely-spread samples, unlike kSumNormalize above.
  /// Computes the dominant eigenvector of a 4x4 symmetric matrix built
  /// from the samples' weighted outer products (see
  /// orientation_averaging.cpp's markleyAverage() for the full algorithm).
  /// Equal-weighted (1/N) today; no per-sample weighting exists anywhere
  /// in this codebase yet.
  kMarkley,
};

/// Result of averaging N orientation samples: the averaged quaternion plus
/// how far each sample deviated from it (angular spread, in degrees) — a
/// quality signal for whether the average is trustworthy, independent of
/// which method produced it. Not yet used to auto-escalate between
/// methods; logged for now.
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
/// std::invalid_argument if samples is empty.
OrientationAveragingResult averageQuaternions(
  const std::vector<tf2::Quaternion> & samples,
  OrientationAveragingMethod method);

/// Angular deviation (degrees) between two quaternions, accounting for the
/// q/-q double-cover of SO(3) (returns the shorter of the two equivalent
/// angles). Exposed as the same formula averageQuaternions() uses
/// internally to compute max_spread_deg/mean_spread_deg, so callers
/// needing a single sample-vs-reference deviation (e.g.
/// CalibrationBroadcasterNode::rejectOutliers) don't have to duplicate it.
double angularDeviationDeg(const tf2::Quaternion & a, const tf2::Quaternion & b);

}  // namespace aruco_perception

#endif  // ARUCO_PERCEPTION__ORIENTATION_AVERAGING_HPP_