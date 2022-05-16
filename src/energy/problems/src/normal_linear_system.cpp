#include "energy/normal_linear_system.hpp"

#include <numeric>

#include "common/settings.hpp"

namespace dsopp {
namespace energy {
namespace {
template <typename Scalar, int SIZE>
Eigen::Vector<Scalar, SIZE> jacobiPreconditioner(Eigen::Matrix<Scalar, SIZE, SIZE> &H) {
  const Scalar kPreconditionerMinValue = 10;
  return (H.diagonal() + Eigen::VectorX<Scalar>::Constant(H.cols(), kPreconditionerMinValue))
      .cwiseSqrt()
      .cwiseInverse();
}
}  // namespace
template <typename Scalar, int SIZE>
void NormalLinearSystem<Scalar, SIZE>::reduce_system(std::vector<int> indices_to_eliminate) {
  CHECK(std::is_sorted(indices_to_eliminate.begin(), indices_to_eliminate.end()));
  std::vector<int> all_indices(static_cast<unsigned int>(b.size()));
  std::iota(all_indices.begin(), all_indices.end(), 0);
  std::vector<int> indecies_to_keep;
  std::set_difference(all_indices.begin(), all_indices.end(), indices_to_eliminate.begin(), indices_to_eliminate.end(),
                      std::inserter(indecies_to_keep, indecies_to_keep.begin()));

  Eigen::Vector<Scalar, Eigen::Dynamic> preconditioner = jacobiPreconditioner(H);
  Eigen::Vector<Scalar, Eigen::Dynamic> preconditioner_inversed = preconditioner.cwiseInverse();
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> hessian_preconditioned =
      preconditioner.asDiagonal() * H * preconditioner.asDiagonal();

  Eigen::Vector<Scalar, Eigen::Dynamic> b_preconditioned = preconditioner.asDiagonal() * b;
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> schur_transform =
      hessian_preconditioned(indecies_to_keep, indices_to_eliminate) *
      hessian_preconditioned(indices_to_eliminate, indices_to_eliminate)
          .completeOrthogonalDecomposition()
          .pseudoInverse();

  hessian_preconditioned =
      hessian_preconditioned(indecies_to_keep, indecies_to_keep).eval() -
      schur_transform * hessian_preconditioned(indecies_to_keep, indices_to_eliminate).eval().transpose();

  b_preconditioned =
      b_preconditioned(indecies_to_keep).eval() - schur_transform * b_preconditioned(indices_to_eliminate).eval();
  hessian_preconditioned = 0.5 * (hessian_preconditioned + hessian_preconditioned.transpose().eval());

  H = preconditioner_inversed(indecies_to_keep).asDiagonal() * hessian_preconditioned *
      preconditioner_inversed(indecies_to_keep).asDiagonal();
  b = preconditioner_inversed(indecies_to_keep).asDiagonal() * b_preconditioned;
}
template <typename Scalar, int SIZE>
Eigen::Vector<Scalar, SIZE> NormalLinearSystem<Scalar, SIZE>::solve() {
  Eigen::Vector<Scalar, Eigen::Dynamic> preconditioner = jacobiPreconditioner(H);
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> h_preconditioned =
      preconditioner.asDiagonal() * H * preconditioner.asDiagonal();
  Eigen::Vector<Scalar, Eigen::Dynamic> x =
      preconditioner.asDiagonal() * h_preconditioned.ldlt().solve(preconditioner.asDiagonal() * b);
  return x;
}

template struct NormalLinearSystem<float, Eigen::Dynamic>;

template struct NormalLinearSystem<double, Eigen::Dynamic>;

template struct NormalLinearSystem<Precision, 6>;
template struct NormalLinearSystem<Precision, 8>;
template struct NormalLinearSystem<Precision, 14>;

}  // namespace energy
}  // namespace dsopp
