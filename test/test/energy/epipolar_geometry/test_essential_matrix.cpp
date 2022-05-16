#include "energy/epipolar_geometry/essential_matrix.hpp"

#include <random>

#include <gtest/gtest.h>

namespace dsopp::energy::epipolar_geometry {

TEST(essential_matrix, essential_matrix) {
  const double kError = 1e-4;
  std::mt19937 rng(static_cast<unsigned int>(time(0)));
  std::uniform_real_distribution<double> angle(-M_PI / 30, M_PI / 30);
  std::uniform_real_distribution<double> t(-30, 30);

  for (int j = 0; j < 100; ++j) {
    Eigen::Vector3d trans(t(rng), t(rng), t(rng));
    Sophus::SO3d q_so3 = Sophus::SO3d::exp(Eigen::Vector3d(angle(rng), angle(rng), angle(rng)));
    Sophus::SE3d pose_gt(q_so3, trans.normalized());
    Eigen::Matrix3d essential_matrix_gt = Sophus::SO3d::hat(pose_gt.translation()) * pose_gt.rotationMatrix();

    Eigen::Matrix3d essential_matrix = transformationToEssentialMatrix(pose_gt);
    auto poses = essentialMatrixToTransformation(essential_matrix);

    EXPECT_LE((essential_matrix_gt.matrix() - essential_matrix.matrix()).norm(),
              essential_matrix_gt.matrix().norm() * kError);

    bool founded = false;
    for (const auto &pose : poses) {
      if ((pose_gt.matrix() - pose.matrix()).norm() < pose_gt.matrix().norm() * kError) {
        founded = true;
      }
    }
    EXPECT_TRUE(founded);
  }
}
}  // namespace dsopp::energy::epipolar_geometry
