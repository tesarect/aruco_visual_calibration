#include "aruco_perception/orientation_averaging.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aruco_perception
{

OrientationAveragingMethod selectAveragingMethod(
  int sum_normalize_priority, int markley_priority)
{
  // Ties broken in kSumNormalize/kMarkley order (sum_normalize preferred
  // when both request the same priority).
  const bool sum_normalize_enabled = sum_normalize_priority > 0;
  const bool markley_enabled = markley_priority > 0;

  if (!sum_normalize_enabled && !markley_enabled) {
    throw std::invalid_argument(
            "No orientation averaging method enabled — set sum_normalize_priority "
            "and/or markley_priority > 0");
  }

  if (sum_normalize_enabled &&
    (!markley_enabled || sum_normalize_priority <= markley_priority))
  {
    return OrientationAveragingMethod::kSumNormalize;
  }
  return OrientationAveragingMethod::kMarkley;
}

namespace
{

/// Component-wise sum of all samples, each flipped to the same hemisphere
/// as samples[0] first (q and -q represent the same rotation, but sum
/// destructively if left in opposite hemispheres), then renormalized.
tf2::Quaternion sumNormalize(const std::vector<tf2::Quaternion> & samples)
{
  tf2::Quaternion sum(0.0, 0.0, 0.0, 0.0);
  const tf2::Quaternion & reference = samples[0];

  for (tf2::Quaternion sample : samples) {
    if (sample.dot(reference) < 0.0) {
      sample = -sample;
    }
    sum += sample;
  }

  sum.normalize();
  return sum;
}

/// Angular deviation (degrees) between two quaternions, accounting for the
/// q/-q double-cover of SO(3) (the shorter of the two equivalent angles).
double angularDeviationDeg(const tf2::Quaternion & a, const tf2::Quaternion & b)
{
  double dot = std::abs(a.dot(b));
  dot = std::min(1.0, std::max(-1.0, dot));  // guard acos domain against fp drift
  return 2.0 * std::acos(dot) * 180.0 / M_PI;
}

}  // namespace

OrientationAveragingResult averageQuaternions(
  const std::vector<tf2::Quaternion> & samples,
  OrientationAveragingMethod method)
{
  if (samples.empty()) {
    throw std::invalid_argument("averageQuaternions requires at least one sample");
  }

  if (method == OrientationAveragingMethod::kMarkley) {
    throw std::invalid_argument(
            "OrientationAveragingMethod::kMarkley is not yet implemented");
  }

  OrientationAveragingResult result;
  result.averaged = sumNormalize(samples);

  double sum_deg = 0.0;
  for (const tf2::Quaternion & sample : samples) {
    const double deviation_deg = angularDeviationDeg(sample, result.averaged);
    result.max_spread_deg = std::max(result.max_spread_deg, deviation_deg);
    sum_deg += deviation_deg;
  }
  result.mean_spread_deg = sum_deg / static_cast<double>(samples.size());

  return result;
}

}  // namespace aruco_perception