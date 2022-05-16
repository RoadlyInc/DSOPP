#include "energy/epipolar_geometry/triangulation.hpp"

#include <random>

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include "common/settings.hpp"

using Precision = dsopp::Precision;

TEST(Triangulation, linearTriangulation) {
  std::mt19937 rng;
  std::uniform_real_distribution<Precision> angle(-1.5, 1.5);
  std::uniform_real_distribution<Precision> coord(0, 10);

  const size_t kTestN = 10;
  for (size_t iteration = 0; iteration < kTestN; ++iteration) {
    Sophus::SO3<Precision> rotation_t_r =
        Sophus::SO3<Precision>::exp(Eigen::Vector3<Precision>(angle(rng), angle(rng), angle(rng)));
    Eigen::Vector3<Precision> translation = Eigen::Vector3<Precision>(coord(rng), coord(rng), coord(rng));
    Eigen::Vector3<Precision> point_3d = Eigen::Vector3<Precision>(coord(rng), coord(rng), coord(rng));

    Sophus::SE3<Precision> t_t_r(rotation_t_r, translation);

    Eigen::Vector3<Precision> reference = point_3d.normalized();
    Eigen::Vector3<Precision> target = (t_t_r * point_3d).normalized();

    Eigen::Vector3<Precision> point_3d_estimated =
        dsopp::energy::epipolar_geometry::triangulation(target, reference, t_t_r.matrix3x4());
    point_3d_estimated = t_t_r.inverse() * point_3d_estimated;
    Precision err = (point_3d - point_3d_estimated).norm() / point_3d.norm();
    EXPECT_LE(err, 1e-5);
  }
}
