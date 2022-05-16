#ifndef GENERIC_CAMERA_TEST_HPP_
#define GENERIC_CAMERA_TEST_HPP_

#include <ceres/jet.h>
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <random>

#include "common/settings.hpp"

using Precision = dsopp::Precision;

template <typename CameraModel>
void TestProjectUnproject(const CameraModel *cam, std::uniform_real_distribution<Precision> &x_coord,
                          std::uniform_real_distribution<Precision> &y_coord) {
  std::mt19937 rng;
  for (int i = 0; i < 100; ++i) {
    Eigen::Vector2<Precision> pt(x_coord(rng), y_coord(rng));
    Eigen::Vector3<Precision> ray = Eigen::Vector3<Precision>::Zero();
    Eigen::Vector2<Precision> ans;

    bool t1 = cam->unproject(pt, ray);

    bool t2 = cam->project(ray, ans);

    EXPECT_TRUE(t1);
    EXPECT_TRUE(t2);

    EXPECT_LE((pt - ans).norm(), 1e-2);
  }
  /** Check minimum and maximum values */
  std::vector<Precision> x = {x_coord.min(), x_coord.max(), x_coord.min(), x_coord.max()};
  std::vector<Precision> y = {y_coord.min(), y_coord.max(), y_coord.max(), y_coord.min()};

  for (size_t i = 0; i < std::min(x.size(), y.size()); ++i) {
    Eigen::Vector2<Precision> pt(x[i], y[i]);
    Eigen::Vector3<Precision> ray = Eigen::Vector3<Precision>::Zero();
    Eigen::Vector2<Precision> ans;

    bool t1 = cam->unproject(pt, ray);

    bool t2 = cam->project(ray, ans);

    EXPECT_TRUE(t1);
    EXPECT_TRUE(t2);

    EXPECT_LE((pt - ans).norm(), 1e-2) << pt.transpose() << " : " << ans.transpose();
  }
}

template <typename CameraModel>
void TestUnprojectProject(const CameraModel *cam, std::uniform_real_distribution<Precision> &x_coord,
                          std::uniform_real_distribution<Precision> &y_coord) {
  std::mt19937 rng;
  for (int i = 0; i < 100; ++i) {
    Eigen::Vector2<Precision> pt(x_coord(rng), y_coord(rng));
    Eigen::Vector3<Precision> ray = Eigen::Vector3<Precision>::Zero();
    bool t = cam->unproject(pt, ray);

    Eigen::Vector3<Precision> ans;

    bool t1 = cam->project(ray, pt);
    bool t2 = cam->unproject(pt, ans);

    EXPECT_TRUE(t);
    EXPECT_TRUE(t1);
    EXPECT_TRUE(t2);

    EXPECT_GT(ans.norm(), 1e-10);
    EXPECT_LE(abs(ray.normalized().cross(ans.normalized()).norm()), 1e-5);
  }
}

template <typename CameraModel>
void TestProjectJacobian(const CameraModel *cam, std::uniform_real_distribution<Precision> &x_coord,
                         std::uniform_real_distribution<Precision> &y_coord) {
  std::mt19937 rng;
  using JT = ceres::Jet<Precision, 3>;
  for (int i = 0; i < 100; ++i) {
    Eigen::Vector2<Precision> pt(x_coord(rng), y_coord(rng));
    Eigen::Vector3<Precision> ray = Eigen::Vector3<Precision>::Zero();
    cam->unproject(pt, ray);

    ray *= static_cast<Precision>(std::abs(rand() % 100));

    Eigen::Vector<JT, 3> ray_ceres = ray.cast<JT>();
    for (int j = 0; j < 3; ++j) ray_ceres(j).v[j] = 1;

    Eigen::Vector<JT, 2> projected_ceres;
    cam->project(ray_ceres, projected_ceres);

    Eigen::Matrix<Precision, 2, 3> projection_jacobian;
    projection_jacobian.setZero();
    Eigen::Vector2<Precision> projected;
    cam->project(ray, projected, projection_jacobian);

    for (int r = 0; r < 2; ++r)
      for (int c = 0; c < 3; ++c) {
        EXPECT_LE(abs(projection_jacobian(r, c) - projected_ceres(r).v[c]), abs(projection_jacobian(r, c)) * 1e-4);
      }
  }
}

template <typename CameraModel>
void TestFocalsEqual(const CameraModel *cam, const Eigen::Vector2<Precision> &focals) {
  EXPECT_EQ(cam->focalX(), focals(0));
  EXPECT_EQ(cam->focalY(), focals(1));
}

#endif  // GENERIC_CAMERA_TEST_HPP_
