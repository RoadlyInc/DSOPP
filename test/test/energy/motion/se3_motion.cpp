#include <memory>
#include <random>

#include <ceres/jet.h>
#include <gtest/gtest.h>
#include <Eigen/Eigen>
#include <sophus/se3.hpp>
#include "common/settings.hpp"
#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"
#include "energy/projector/camera_reproject.hpp"

using Precision = dsopp::Precision;
using namespace dsopp::energy::motion;
using JT = ceres::Jet<double, 7>;
using dsopp::operator"" _p;

template <Motion Motion>
struct foo {};
TEST(SE3Motion, MotionConcept) { foo<SE3<Precision>>(); }

class SE3MotionTest : public ::testing::Test {
 public:
  void SetUp() {
    // srand(static_cast<unsigned int>(time(0)));

    tangent.setRandom();

    tangentEps.setZero();
    for (int i = 0; i < 6; ++i) tangentEps(i).v[i] = 1;

    t_b_a = SE3<Precision>::exp(tangent);
    t_b_a_JT = t_b_a.cast<JT>();
    t_b_a_eps = SE3<JT>::exp(tangentEps);

    x = Eigen::Vector3<Precision>::Random();
  }

  void TearDown() {}
  Eigen::Vector<Precision, 6> tangent;
  Eigen::Vector<JT, 6> tangentEps;
  SE3<Precision> t_b_a;
  SE3<JT> t_b_a_JT;
  SE3<JT> t_b_a_eps;
  Eigen::Vector3<Precision> x;

  Precision eps = 1e-5_p;
};

TEST_F(SE3MotionTest, RightBoxPlusJacobian) {
  Precision idepth = 0.1_p;
  SE3<JT> t_b_a_ceres = t_b_a.cast<JT>() * t_b_a_eps;

  Eigen::Vector<JT, 3> right_jacobian_ceres = t_b_a_ceres.so3() * x.cast<JT>() + JT(idepth) * t_b_a_ceres.translation();
  auto right_jacobian = t_b_a.rightBoxPlusJacobian(x, idepth);

  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 6; ++c) {
      EXPECT_LE(abs(right_jacobian(r, c) - right_jacobian_ceres(r).v[c]), std::abs(right_jacobian(r, c)) * eps);
    }
  }
}

TEST_F(SE3MotionTest, ApplyPropagationOfUncertainty) {
  using JSigma = ceres::Jet<Precision, 12>;

  tangent.setRandom();
  SE3<Precision> t_w_1 = SE3<Precision>::exp(tangent);
  tangent.setRandom();
  SE3<Precision> t_w_2 = SE3<Precision>::exp(tangent);

  Eigen::Vector<JSigma, 6> tangent_t_w_1_eps;
  tangent_t_w_1_eps.setZero();
  for (int i = 0; i < 6; ++i) tangent_t_w_1_eps(i).v[i] = 1;
  auto t_w_1_eps = SE3<JSigma>::exp(tangent_t_w_1_eps);

  Eigen::Vector<JSigma, 6> tangent_t_w_2_eps;
  tangent_t_w_2_eps.setZero();
  for (int i = 0; i < 6; ++i) tangent_t_w_2_eps(i).v[i + 6] = 1;
  auto t_w_2_eps = SE3<JSigma>::exp(tangent_t_w_2_eps);

  auto jacobian_ceres_jet =
      ((t_w_1.inverse() * t_w_2).inverse() * (t_w_1 * t_w_1_eps).inverse() * t_w_2 * t_w_2_eps).log();
  Eigen::Matrix<Precision, 6, 12> jacobian_ceres;
  for (int r = 0; r < 6; ++r) {
    jacobian_ceres.row(r) = jacobian_ceres_jet(r).v;
  }
  Eigen::Matrix<Precision, 6, 6> sigma_11 = Eigen::Matrix<Precision, 6, 6>::Random();
  Eigen::Matrix<Precision, 6, 6> sigma_12 = Eigen::Matrix<Precision, 6, 6>::Random();
  Eigen::Matrix<Precision, 6, 6> sigma_22 = Eigen::Matrix<Precision, 6, 6>::Random();
  Eigen::Matrix<Precision, 12, 12> sigma;
  sigma.block<6, 6>(0, 0).noalias() = sigma_11;
  sigma.block<6, 6>(0, 6).noalias() = sigma_12;
  sigma.block<6, 6>(6, 0).noalias() = sigma_12.transpose();
  sigma.block<6, 6>(6, 6).noalias() = sigma_22;

  Eigen::Matrix<Precision, 6, 6> sigma_ceres = jacobian_ceres * sigma * jacobian_ceres.transpose();
  Eigen::Matrix<Precision, 6, 6> sigma_eigen =
      SE3<Precision>::relativeTransformationUncertainty(t_w_1, t_w_2, sigma_11, sigma_22, sigma_12);

  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 6; ++c) {
      EXPECT_LE(abs(sigma_ceres(r, c) - sigma_eigen(r, c)), eps);
    }
  }
}

TEST_F(SE3MotionTest, LeftBoxPlusJacobian) {
  Precision idepth = 0.1_p;
  auto t_b_a_ceres = t_b_a_eps * t_b_a_JT;

  Eigen::Vector<JT, 3> left_jacobian_ceres = t_b_a_ceres.so3() * x.cast<JT>() + JT(idepth) * t_b_a_ceres.translation();
  auto left_jacobian = t_b_a.leftBoxPlusJacobian(x, idepth);

  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 6; ++c) {
      EXPECT_LE(abs(left_jacobian(r, c) - left_jacobian_ceres(r).v[c]), eps);
    }
  }
}

TEST_F(SE3MotionTest, QuaternionTranslationRightJacobian) {
  t_b_a_JT = t_b_a.cast<JT>();
  Precision idepth = 0.235_p;
  for (int i = 0; i < 7; ++i) t_b_a_JT.data()[i].v[i] = 1;

  Eigen::Vector<JT, 4> x_homo = x.cast<JT>().homogeneous();
  x_homo(3) = JT(idepth);

  auto jacobian_ceres = t_b_a_JT * x_homo;

  auto jacobian = t_b_a.actionJacobianQuaternionTranslation(x, idepth);

  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 7; ++c) {
      EXPECT_LE(abs(jacobian(r, c) - jacobian_ceres(r).v[c]), eps);
    }
  }
}

TEST_F(SE3MotionTest, Parameters) {
  Eigen::Vector<Precision, 7> params = 100 * Eigen::Vector<Precision, 7>::Random();
  SE3<Precision> motion;
  motion.setParameters(params);
  EXPECT_LE((params - motion.parameters()).norm(), eps);
}
