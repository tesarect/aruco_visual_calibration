#include "aruco_perception/orientation_averaging.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Eigen/Eigenvalues>

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
      // Component-wise negation instead of unary operator- (which is
      // ambiguous against btQuaternion's inherited operator- overloads):
      // flips sample to the same hemisphere as reference, since q and -q
      // represent the same rotation but would sum destructively otherwise.
      sample.setValue(-sample.x(), -sample.y(), -sample.z(), -sample.w());
    }
    sum += sample;
  }

  sum.normalize();
  return sum;
}

/// Markley's method (Markley, Cheng, Crassidis, Oshman, "Averaging
/// Quaternions," JGCD 2007) — the quaternion that minimizes the weighted
/// sum of squared chordal (Euclidean, in R^4) distances to every sample is
/// the eigenvector of the LARGEST eigenvalue of M = sum_i w_i * (q_i *
/// q_i^T), a 4x4 symmetric matrix built from the samples' outer products.
/// Unlike sumNormalize()'s pre-sum hemisphere flip, this metric is
/// naturally insensitive to the q/-q double-cover (squaring the dot
/// product in the underlying optimization makes q_i and -q_i contribute
/// identically) — no per-sample flip needed before accumulating M, only a
/// single sign-ambiguity resolution on the FINAL result (the dominant
/// eigenvector itself is only defined up to an overall sign).
///
/// Equal weights (w_i = 1/N) — matches every existing averageQuaternions()
/// call site, none of which pass or track per-sample weights today.
/// Weighting cancels out of "which eigenvector is dominant" for a uniform
/// weight anyway (only relative weights matter), so 1/N vs. un-normalized
/// 1 makes no difference here — kept as 1/N for clarity against the
/// paper's own formulation.
tf2::Quaternion markleyAverage(const std::vector<tf2::Quaternion> & samples)
{
  const double weight = 1.0 / static_cast<double>(samples.size());

  Eigen::Matrix4d m = Eigen::Matrix4d::Zero();
  for (const tf2::Quaternion & sample : samples) {
    // tf2::Quaternion's own accessor order (x, y, z, w) — component order
    // doesn't matter mathematically as long as it's applied consistently
    // between q_i's construction here and the eigenvector's decode below.
    const Eigen::Vector4d q(sample.x(), sample.y(), sample.z(), sample.w());
    m += weight * (q * q.transpose());
  }

  // SelfAdjointEigenSolver (not the general EigenSolver): exploits M's
  // symmetry for a numerically robust, efficient solve — the correct
  // Eigen class for a symmetric/self-adjoint matrix. Eigenvalues are
  // returned in ASCENDING order, so the dominant eigenvector (largest
  // eigenvalue) is the LAST column, index 3, for a 4x4 matrix.
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(m);
  Eigen::Vector4d dominant = solver.eigenvectors().col(3);

  // Sign ambiguity: the eigenvector is only defined up to an overall sign
  // (v and -v are the same physical rotation) — resolve against samples[0]
  // the same way sumNormalize() resolves its own hemisphere ambiguity, so
  // both methods are consistent/comparable in which hemisphere they land.
  const tf2::Quaternion & reference = samples[0];
  const double dot =
    dominant.x() * reference.x() + dominant.y() * reference.y() +
    dominant.z() * reference.z() + dominant.w() * reference.w();
  if (dot < 0.0) {
    dominant = -dominant;
  }

  tf2::Quaternion result(dominant.x(), dominant.y(), dominant.z(), dominant.w());
  result.normalize();
  return result;
}

}  // namespace

double angularDeviationDeg(const tf2::Quaternion & a, const tf2::Quaternion & b)
{
  double dot = std::abs(a.dot(b));
  dot = std::min(1.0, std::max(-1.0, dot));  // guard acos domain against fp drift
  return 2.0 * std::acos(dot) * 180.0 / M_PI;
}

OrientationAveragingResult averageQuaternions(
  const std::vector<tf2::Quaternion> & samples,
  OrientationAveragingMethod method)
{
  if (samples.empty()) {
    throw std::invalid_argument("averageQuaternions requires at least one sample");
  }

  OrientationAveragingResult result;
  result.averaged = method == OrientationAveragingMethod::kMarkley ?
    markleyAverage(samples) : sumNormalize(samples);

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