#include <memory>

#include <ceres/jet.h>
#include <gtest/gtest.h>
#include "common/settings.hpp"
#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/camera_model/pinhole/simple_radial.hpp"
#include "energy/motion/se3_motion.hpp"

#include "energy/projector/camera_projector.hpp"

namespace dsopp::energy::reprojection {

using Jet = ceres::Jet<double, 4>;

class testCameraProjector : public ::testing::Test {
 public:
  const Precision kBorderSize = 50;
  void SetUp() override {
    /** to be sure that rolling shutter projection would not be out of image */
    Eigen::Vector2<Precision> focals = Eigen::Vector2<Precision>(448, 448);
    Eigen::Vector2<Precision> img_size = Eigen::Vector2<Precision>(1280, 640);
    pinhole_cam_ =
        std::make_unique<model::PinholeCamera<Precision>>(img_size, focals, Eigen::Vector2<Precision>(640, 320));
    pinhole_cam_level2_ = std::make_unique<model::PinholeCamera<Precision>>(
        img_size, focals, Eigen::Vector2<Precision>(640, 320), time::duration(), 3);

    radial_cam_ = std::make_unique<model::SimpleRadialCamera<Precision>>(
        img_size, 509.022278, Eigen::Vector2<Precision>(640, 360), 0.022297781517582316, -0.01355120683821621);

    radial_cam_level2_ = std::make_unique<model::SimpleRadialCamera<Precision>>(
        img_size, 509.022278, Eigen::Vector2<Precision>(640, 360), 0.022297781517582316, -0.01355120683821621,
        time::duration(), 3);

    pX = std::uniform_real_distribution<Precision>(kBorderSize, img_size(0) - 1 - kBorderSize);
    pY = std::uniform_real_distribution<Precision>(kBorderSize, img_size(1) - 1 - kBorderSize);
    pIdepth = std::uniform_real_distribution<Precision>(0.01_p, 1);

    left_velocity = Eigen::Vector<Precision, 6>::Random().normalized() * 1e-5;
    right_velocity = Eigen::Vector<Precision, 6>::Random().normalized() * 1e-5;
  }
  void TearDown() override {}

 protected:
  Eigen::Vector<Precision, 6> left_velocity, right_velocity;
  std::unique_ptr<model::PinholeCamera<Precision>> pinhole_cam_;
  std::unique_ptr<model::SimpleRadialCamera<Precision>> radial_cam_;

  std::unique_ptr<model::PinholeCamera<Precision>> pinhole_cam_level2_;
  std::unique_ptr<model::SimpleRadialCamera<Precision>> radial_cam_level2_;

  const int kTestNumber_ = 10;
  std::mt19937 rng;
  std::uniform_real_distribution<Precision> pX;
  std::uniform_real_distribution<Precision> pY;
  std::uniform_real_distribution<Precision> pIdepth;
};

template <class Model, class Projector, class ProjectorJet>
void test_CameraModel_unprojectJacobian(Projector &projector, ProjectorJet &projector_ceres, Model &model,
                                        std::uniform_real_distribution<Precision> &pX,
                                        std::uniform_real_distribution<Precision> &pY,
                                        std::uniform_real_distribution<Precision> &pIdepth, std::mt19937 &rng,
                                        const int test_number) {
  for (int i = 0; i < test_number; ++i) {
    Eigen::Vector2<Precision> pixel(pX(rng), pY(rng));
    Precision idepth = pIdepth(rng);

    Eigen::Vector3<Precision> ray;

    model.unproject(pixel, ray);
    Eigen::Vector4<Precision> point;
    point << ray, idepth;

    Eigen::Vector<Jet, 4> point_ceres = point.cast<Jet>();
    for (int j = 0; j < 4; ++j) point_ceres(j).v[j] = 1;

    Eigen::Matrix<Precision, 2, 4> jacobian;
    Eigen::Vector<Precision, 2> projector_pixel = Eigen::Vector2<Precision>::Zero();
    bool success = projector.project(point, projector_pixel, jacobian);

    Eigen::Vector<Jet, 2> pixel_ceres;
    success &= projector_ceres.project(point_ceres, pixel_ceres);

    if (!success) {
      i -= 1;
      continue;
    }

    for (int j = 0; j < 2; ++j) EXPECT_LE(abs(point(j) - point_ceres(j).a), 1e-8);

    Eigen::Matrix<Precision, 2, 4> ceres_jacobian;
    for (int r = 0; r < 2; ++r) ceres_jacobian.row(r) = pixel_ceres(r).v.cast<Precision>();
    EXPECT_LE((jacobian - ceres_jacobian).norm(), ceres_jacobian.norm() * 1e-3);
  }
}

template <class Model>
void run_SE3Tests(Model &model, std::uniform_real_distribution<Precision> &pX,
                  std::uniform_real_distribution<Precision> &pY, std::uniform_real_distribution<Precision> &pIdepth,
                  std::mt19937 &rng, const int test_number) {
  energy::motion::SE3<Precision> t_t_r;
  energy::motion::SE3<Jet> t_t_r_ceres = t_t_r.cast<Jet>();

  Projector<Precision, Model, decltype(t_t_r)> projector(model, t_t_r);
  Projector<Jet, Model, decltype(t_t_r_ceres)> projector_ceres(model, t_t_r_ceres);

  test_CameraModel_unprojectJacobian(projector, projector_ceres, model, pX, pY, pIdepth, rng, test_number);
}

TEST_F(testCameraProjector, Pinhole_SE3_jacobian_unproject) {
  run_SE3Tests(*pinhole_cam_, pX, pY, pIdepth, rng, kTestNumber_);
}

TEST_F(testCameraProjector, SimpleRadial_SE3_jacobian_unproject) {
  run_SE3Tests(*radial_cam_, pX, pY, pIdepth, rng, kTestNumber_);
}

}  // namespace dsopp::energy::reprojection
