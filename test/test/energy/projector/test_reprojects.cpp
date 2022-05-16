#include <memory>
#include <numbers>

#include <ceres/jet.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <sophus/se3.hpp>

#include "common/settings.hpp"
#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/camera_model/pinhole/simple_radial.hpp"
#include "energy/motion/se3_motion.hpp"

#include "energy/projector/camera_reproject.hpp"

namespace dsopp::energy::model {

const int N = 8;

class testPinholeReproject : public ::testing::Test {
 public:
  using Vec3 = Eigen::Matrix<Precision, 3, 1>;
  using Vec2 = Eigen::Matrix<Precision, 2, 1>;

 protected:
  void SetUp() override {
    const Precision kBorderSize = 4;
    Eigen::Vector2<Precision> focals = Eigen::Vector2<Precision>(448, 448);
    Eigen::Vector2<Precision> target_focals = Eigen::Vector2<Precision>(500, 400);
    Eigen::Vector2<Precision> img_size = Eigen::Vector2<Precision>(1280, 640);
    cam_ = std::make_unique<PinholeCamera<Precision>>(img_size, focals, Eigen::Vector2<Precision>(640, 320));

    target_cam_ =
        std::make_unique<PinholeCamera<Precision>>(img_size, target_focals, Eigen::Vector2<Precision>(600, 300));

    pinhole_cam_level2_ = std::make_unique<model::PinholeCamera<Precision>>(
        img_size, focals, Eigen::Vector2<Precision>(640, 320), time::duration(), 3);

    radial_cam_ = std::make_unique<SimpleRadialCamera<Precision>>(
        img_size, 745.088891888 * 1.5, Eigen::Vector2<Precision>(640, 360), 0.017259368893, -0.017259368893);

    target_radial_cam_ = std::make_unique<SimpleRadialCamera<Precision>>(
        img_size, 509.022278, Eigen::Vector2<Precision>(600, 300), 0.022297781517582316, -0.01355120683821621);

    radial_cam_level2_ = std::make_unique<model::SimpleRadialCamera<Precision>>(
        img_size, 745.088891888 * 1.5, Eigen::Vector2<Precision>(640, 360), 0.017259368893, -0.017259368893,
        time::duration(), 3);

    pX = std::uniform_real_distribution<Precision>(kBorderSize, img_size(0) - 1 - kBorderSize);
    pY = std::uniform_real_distribution<Precision>(kBorderSize, img_size(1) - 1 - kBorderSize);
    pIdepth = std::uniform_real_distribution<Precision>(0.01_p, 1);
    angle = std::uniform_real_distribution<Precision>(-std::numbers::pi_v<Precision> / 30,
                                                      std::numbers::pi_v<Precision> / 30);

    pose2 = Sophus::SE3<Precision>(Sophus::SO3<Precision>::exp(Eigen::Vector3<Precision>(0, angle(rng), 0)),
                                   Eigen::Vector3<Precision>(0, 0, -1));
  }
  void TearDown() override {}

  Sophus::SE3<Precision> pose1, pose2;
  std::unique_ptr<PinholeCamera<Precision>> cam_;
  std::unique_ptr<PinholeCamera<Precision>> target_cam_;
  std::unique_ptr<SimpleRadialCamera<Precision>> radial_cam_;
  std::unique_ptr<SimpleRadialCamera<Precision>> target_radial_cam_;

  std::unique_ptr<model::PinholeCamera<Precision>> pinhole_cam_level2_;
  std::unique_ptr<model::SimpleRadialCamera<Precision>> radial_cam_level2_;

  const int testNumber_ = 10;
  std::mt19937 rng;
  std::uniform_real_distribution<Precision> pX;
  std::uniform_real_distribution<Precision> pY;
  std::uniform_real_distribution<Precision> pIdepth;
  std::uniform_real_distribution<Precision> angle;
};

TEST_F(testPinholeReproject, multipleReprojectTest) {
  for (auto pose : {pose1, pose2}) {
    energy::reprojection::ArrayReprojector<Precision, energy::model::PinholeCamera<Precision>,
                                           energy::motion::SE3<Precision>>
        reprojector(*cam_, pose);

    for (int i = 0; i < testNumber_; ++i) {
      Eigen::Vector2<Precision> pixel(pX(rng), pY(rng));
      Precision idepth = pIdepth(rng);

      Eigen::Vector3<Precision> ray;

      cam_->unproject(pixel, ray);
      ray = pose.so3() * ray + idepth * pose.translation();
      Eigen::Vector2<Precision> pixel_target;
      cam_->project(ray, pixel_target);

      Eigen::Vector2<Precision> reprojection_target;
      reprojector.reproject(pixel, idepth, reprojection_target);
      CHECK_LE((pixel_target - reprojection_target).norm(), 1e-3);
    }
  }
}

TEST_F(testPinholeReproject, patternReprojectTest) {
  for (auto pose : {pose1, pose2}) {
    energy::reprojection::ArrayReprojector<Precision, energy::model::PinholeCamera<Precision>,
                                           energy::motion::SE3<Precision>>
        reprojector(*cam_, pose);

    Precision idepth = pIdepth(rng);
    Eigen::Matrix<Precision, 2, N> reference_pattern;
    for (int i = 0; i < N; ++i) {
      Eigen::Vector2<Precision> pixel(pX(rng), pY(rng));

      reference_pattern.col(i) = pixel;

      Eigen::Vector3<Precision> ray;

      bool success = cam_->unproject(pixel, ray);
      ray = pose.so3() * ray + idepth * pose.translation();
      Eigen::Vector2<Precision> pixel_target;
      success = success && cam_->project(ray, pixel_target);
      if (!success) {
        i = -1;
        idepth = pIdepth(rng);
      }
    }
    Eigen::Matrix<Precision, 2, N> target_pattern;

    reprojector.reprojectPattern(reference_pattern, idepth, target_pattern);

    for (int i = 0; i < N; ++i) {
      Eigen::Vector2<Precision> pixel = reference_pattern.col(i);

      Eigen::Vector3<Precision> ray;

      cam_->unproject(pixel, ray);
      ray = pose.so3() * ray + idepth * pose.translation();
      Eigen::Vector2<Precision> pixel_target;
      cam_->project(ray, pixel_target);

      Eigen::Vector2<Precision> reprojection_target = target_pattern.col(i);

      CHECK_LE((pixel_target - reprojection_target).norm(), 1e-3);
    }
  }
}

template <class Model>
void testPatternReprojectWithJacobians(Model &reference_model, Model &target_model, const Sophus::SE3<Precision> &pose1,
                                       const Sophus::SE3<Precision> &pose2,
                                       std::uniform_real_distribution<Precision> &pX,
                                       std::uniform_real_distribution<Precision> &pY,
                                       std::uniform_real_distribution<Precision> &pIdepth, std::mt19937 &rng) {
  using Jet = ceres::Jet<Precision, 6 + 1>;
  Eigen::Vector<Jet, 6> infinitesimal = Eigen::Vector<Jet, 6>::Zero();
  for (int i = 0; i < 6; ++i) infinitesimal(i).v[i] = 1;

  for (auto pose : {pose2, pose1}) {
    energy::reprojection::ArrayReprojector<Precision, Model, energy::motion::SE3<Precision>> reprojector(
        reference_model, target_model, pose);

    energy::motion::SE3<Jet> pose_ceres = energy::motion::SE3<Jet>::exp(infinitesimal) * pose.template cast<Jet>();
    energy::reprojection::ArrayReprojector<Jet, Model, energy::motion::SE3<Jet>> reprojector_ceres(
        reference_model, target_model, pose_ceres);

    Precision idepth = pIdepth(rng);
    Eigen::Matrix<Precision, 2, N> reference_pattern;
    for (int i = 0; i < N; ++i) {
      Eigen::Vector2<Precision> pixel(pX(rng), pY(rng));
      reference_pattern.col(i) = pixel;

      Eigen::Vector3<Precision> ray;

      bool success = reference_model.unproject(pixel, ray);
      ray = pose.so3() * ray + idepth * pose.translation();
      Eigen::Vector2<Precision> pixel_target;
      success = success && target_model.project(ray, pixel_target);
      if (!success) {
        i = -1;
        idepth = pIdepth(rng);
      }
    }
    Eigen::Matrix<Jet, 2, N> target_pattern_ceres;
    Jet idepth_jet = Jet(idepth);
    idepth_jet.v[6] = 1;
    reprojector_ceres.reprojectPattern(reference_pattern.cast<Jet>().eval(), idepth_jet, target_pattern_ceres);

    Eigen::Matrix<Precision, 2, N> target_pattern;
    Eigen::Vector<Precision, N> d_u_idepth;
    Eigen::Vector<Precision, N> d_v_idepth;
    Eigen::Matrix<Precision, N, Sophus::SE3<Precision>::DoF> d_u_tReferenceTarget;
    Eigen::Matrix<Precision, N, Sophus::SE3<Precision>::DoF> d_v_tReferenceTarget;
    reprojector.reprojectPattern(reference_pattern, idepth, target_pattern, d_u_idepth, d_v_idepth,
                                 d_u_tReferenceTarget, d_v_tReferenceTarget);

    CHECK_LE((target_pattern.template cast<Jet>() - target_pattern_ceres).norm().a, 1e-2);

    for (int i = 0; i < N; i++) {
      EXPECT_LE(abs(target_pattern_ceres(0, i).v[6] - d_u_idepth(i)), abs(d_u_idepth(i)) * 5e-3 + 1e-1);
      EXPECT_LE(abs(target_pattern_ceres(1, i).v[6] - d_v_idepth(i)), abs(d_v_idepth(i)) * 5e-3 + 1e-1);

      EXPECT_LE((d_u_tReferenceTarget.row(i).transpose() - target_pattern_ceres(0, i).v.head<6>()).norm(),
                d_u_tReferenceTarget.row(i).norm() * 5e-3 + 1e-1);
      EXPECT_LE((d_v_tReferenceTarget.row(i).transpose() - target_pattern_ceres(1, i).v.head<6>()).norm(),
                d_v_tReferenceTarget.row(i).norm() * 5e-3 + 1e-1);
    }
  }
}

TEST_F(testPinholeReproject, patternReprojectWithJacobiansPinhole) {
  testPatternReprojectWithJacobians<energy::model::PinholeCamera<Precision>>(*cam_, *cam_, pose1, pose2, pX, pY,
                                                                             pIdepth, rng);
}

TEST_F(testPinholeReproject, patternReprojectWithJacobiansTwoModelsPinhole) {
  testPatternReprojectWithJacobians<energy::model::PinholeCamera<Precision>>(*cam_, *target_cam_, pose1, pose2, pX, pY,
                                                                             pIdepth, rng);
}

TEST_F(testPinholeReproject, patternReprojectWithJacobiansSimpleRadial) {
  testPatternReprojectWithJacobians<energy::model::SimpleRadialCamera<Precision>>(*radial_cam_, *radial_cam_, pose1,
                                                                                  pose2, pX, pY, pIdepth, rng);
}

TEST_F(testPinholeReproject, patternReprojectWithJacobiansTwoModelsSimpleRadial) {
  testPatternReprojectWithJacobians<energy::model::SimpleRadialCamera<Precision>>(*radial_cam_, *target_radial_cam_,
                                                                                  pose1, pose2, pX, pY, pIdepth, rng);
}

template <class Reprojector>
void compareTwoReprojectors(Reprojector &reprojector_with_one_model, Reprojector &reprojector_with_two_models,
                            std::uniform_real_distribution<Precision> &pX,
                            std::uniform_real_distribution<Precision> &pY,
                            std::uniform_real_distribution<Precision> &pIdepth, const int testNumber,
                            std::mt19937 &rng) {
  for (int i = 0; i < testNumber; ++i) {
    Eigen::Vector2<Precision> pixel(pX(rng), pY(rng));
    Precision idepth = pIdepth(rng);

    Eigen::Vector2<Precision> reprojection_one_model;
    reprojector_with_one_model.reproject(pixel, idepth, reprojection_one_model);

    Eigen::Vector2<Precision> reprojection_two_models;
    reprojector_with_two_models.reproject(pixel, idepth, reprojection_two_models);
    CHECK_LE((reprojection_one_model - reprojection_two_models).norm(), 1e-3);
  }
}

template <class Model>
void testTwoEquivalentCameraModels(Model &model, const Sophus::SE3<Precision> &pose,
                                   std::uniform_real_distribution<Precision> &pX,
                                   std::uniform_real_distribution<Precision> &pY,
                                   std::uniform_real_distribution<Precision> &pIdepth, const int testNumber,
                                   std::mt19937 &rng) {
  energy::reprojection::ArrayReprojector<Precision, Model, energy::motion::SE3<Precision>> reprojector_with_one_model(
      model, pose);
  energy::reprojection::ArrayReprojector<Precision, Model, energy::motion::SE3<Precision>> reprojector_with_two_models(
      model, model, pose);
  compareTwoReprojectors(reprojector_with_one_model, reprojector_with_two_models, pX, pY, pIdepth, testNumber, rng);
}

TEST_F(testPinholeReproject, TwoEquivalentPinholeCameras) {
  testTwoEquivalentCameraModels(*cam_, pose2, pX, pY, pIdepth, testNumber_, rng);
}

TEST_F(testPinholeReproject, TwoEquivalentSimpleRadialCameras) {
  testTwoEquivalentCameraModels(*radial_cam_, pose2, pX, pY, pIdepth, testNumber_, rng);
}

}  // namespace dsopp::energy::model
