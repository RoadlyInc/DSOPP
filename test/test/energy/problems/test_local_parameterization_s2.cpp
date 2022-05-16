#include "energy/problems/local_parameterization_s2.hpp"

#include <gtest/gtest.h>

namespace dsopp {
namespace energy {
namespace problem {
TEST(test_local_parameterization_s2, test_local_parameterization_s2) {
  for (double delta_theta = 0; delta_theta <= M_2_PI; delta_theta += 0.01 * M_PI) {
    for (double delta_phi = 0; delta_phi <= M_2_PI; delta_phi += 0.01 * M_PI) {
      LocalParameterizationS2 local_parameterization_s2;

      Eigen::Vector3<double> vector = Eigen::Vector3<double>::Random();
      vector.stableNormalize();

      Eigen::Vector2<double> delta(delta_theta, delta_phi);

      Eigen::Vector3<double> vector_plus_delta;
      local_parameterization_s2.Plus(vector.data(), delta.data(), vector_plus_delta.data());

      ASSERT_TRUE(abs(vector_plus_delta.norm() - 1) < 1e-15);

      Eigen::Matrix<double, 3, 2, Eigen::RowMajor> jacobian;
      local_parameterization_s2.ComputeJacobian(vector.data(), jacobian.data());

      delta = Eigen::Vector2<double>::Random() * 1e-5;
      local_parameterization_s2.Plus(vector.data(), delta.data(), vector_plus_delta.data());
      ASSERT_NEAR((vector + jacobian * delta - vector_plus_delta).norm(), 0, 1e-9);
    }
  }
}
}  // namespace problem
}  // namespace energy
}  // namespace dsopp
