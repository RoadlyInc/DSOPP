#ifndef DSOPP_ENERGY_CAMERA_MODELS_POLYNOMIAL_SOLVER_HPP_
#define DSOPP_ENERGY_CAMERA_MODELS_POLYNOMIAL_SOLVER_HPP_

#include <Eigen/Dense>

#include "common/settings.hpp"

namespace dsopp::energy::model::polynomial_solver {

template <class Scalar>
Scalar minimalPositiveRoot(Eigen::VectorX<Scalar> polynomial) {
  const Precision kEps = 1e-8_p;

  polynomial /= polynomial.norm();

  long n = polynomial.rows() - 1;
  while (abs(polynomial[n]) < Scalar(kEps)) {
    n--;
  }
  if (n == 0) {
    return Scalar(0);
  }
  Eigen::MatrixX<Scalar> companion(n, n);
  companion.setZero();
  companion.block(0, n - 1, n, 1) = -polynomial.head(n) / polynomial[n];
  companion.block(1, 0, n - 1, n - 1).diagonal().setOnes();

  auto lambda = (companion.eigenvalues().eval());

  Scalar ans = Scalar(std::numeric_limits<Precision>::infinity());

  for (int i = 0; i < lambda.rows(); ++i) {
    auto l = lambda[i];

    if (abs(l.imag()) > Scalar(Eigen::NumTraits<Precision>::epsilon()) * abs(l.real()) || l.real() < Scalar(0.0))
      continue;
    ans = std::min(ans, l.real());
  }

  return ans;
}

}  // namespace dsopp::energy::model::polynomial_solver

#endif  // DSOPP_ENERGY_CAMERA_MODELS_POLYNOMIAL_SOLVER_HPP_
