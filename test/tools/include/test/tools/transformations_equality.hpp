#ifndef DSOPP_TEST_TOOLS_TRANSFORMATIONS_EQUALITY_HPP_
#define DSOPP_TEST_TOOLS_TRANSFORMATIONS_EQUALITY_HPP_

#include <numbers>

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include "common/settings.hpp"

namespace dsopp {
Precision angleError(const Sophus::SE3<Precision>& T1, const Sophus::SE3<Precision>& T2) {
  Sophus::SE3 diff = T1 * T2.inverse();

  return 180 * diff.so3().log().norm() / std::numbers::pi_v<Precision>;
}

Precision distanceError(const Sophus::SE3<Precision>& T1, const Sophus::SE3<Precision>& T2) {
  Sophus::SE3 diff = T1 * T2.inverse();
  Precision max_dist = std::max(T1.translation().norm(), T2.translation().norm());
  if (max_dist > 1e-5) {
    return diff.translation().norm() / max_dist;
  } else {
    return 0;
  }
}

void assertEqual(const Sophus::SE3<Precision>& T1, const Sophus::SE3<Precision>& T2, Precision maxDist,
                 Precision maxAngle) {
  EXPECT_LE(angleError(T1, T2), (180 * maxAngle / M_PI));
  EXPECT_LE(distanceError(T1, T2), maxDist);
}
}  // namespace dsopp

#endif  // DSOPP_TEST_TOOLS_TRANSFORMATIONS_EQUALITY_HPP_
