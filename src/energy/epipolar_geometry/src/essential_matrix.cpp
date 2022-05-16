#include "energy/epipolar_geometry/essential_matrix.hpp"

namespace dsopp {
namespace energy {
namespace epipolar_geometry {

std::vector<Sophus::SE3d> essentialMatrixToTransformation(const Eigen::Matrix3d &essential_matrix) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd_holder(essential_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::MatrixXd U = svd_holder.matrixU();
  Eigen::MatrixXd V = svd_holder.matrixV();
  U *= U.determinant();
  V *= V.determinant();
  Eigen::Matrix3d W;
  W << 0, -1, 0, 1, 0, 0, 0, 0, 1;

  std::vector<Eigen::Matrix3d> rotations = {U * W * V.transpose(), U * W * V.transpose(),
                                            U * W.transpose() * V.transpose(), U * W.transpose() * V.transpose()};
  std::vector<Eigen::Vector3d> translations = {U.col(2), -U.col(2), U.col(2), -U.col(2)};
  std::vector<Sophus::SE3d> result;
  for (size_t i = 0; i < rotations.size(); i++) {
    result.emplace_back(rotations[i], translations[i]);
  }

  return result;
}

Eigen::Matrix3d transformationToEssentialMatrix(const Sophus::SE3d &t_t_r) {
  return Sophus::SO3d::hat(t_t_r.translation()) * t_t_r.rotationMatrix();
}

}  // namespace epipolar_geometry
}  // namespace energy
}  // namespace dsopp
