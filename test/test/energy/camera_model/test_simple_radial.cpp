#include "energy/camera_model/pinhole/simple_radial.hpp"

#include "generic_camera_test.hpp"

#include <gtest/gtest.h>
#include <Eigen/Eigen>
#include <random>

using SimpleRadialCamera = dsopp::energy::model::SimpleRadialCamera<Precision>;
using dsopp::operator"" _p;

class testSimpleRadialCamera : public ::testing::Test {
 public:
  void SetUp() {
    const Precision kBorderSize = SimpleRadialCamera::kBorderSize;
    img_size = Eigen::Vector2<Precision>(1280, 720);
    focals = Eigen::Vector2<Precision>(509.022278, 509.022278);
    Eigen::Vector2<Precision> center(640, 360);
    Precision k1 = 0.022297781517582316_p, k2 = -0.01355120683821621_p;

    cam_ = std::make_unique<SimpleRadialCamera>(img_size, focals(0), center, k1, k2);

    Precision eps = 1e-2_p;
    pX = std::uniform_real_distribution<Precision>(kBorderSize + eps, img_size(0) - 1 - kBorderSize - eps);
    pY = std::uniform_real_distribution<Precision>(kBorderSize + eps, img_size(1) - 1 - kBorderSize - eps);
  }

  void TearDown() {}

  std::unique_ptr<SimpleRadialCamera> cam_;
  Eigen::Vector2<Precision> focals;
  Eigen::Vector2<Precision> img_size;

  std::uniform_real_distribution<Precision> pX;
  std::uniform_real_distribution<Precision> pY;
};

TEST_F(testSimpleRadialCamera, TestFocalLengths) { TestFocalsEqual(cam_.get(), focals); }
TEST_F(testSimpleRadialCamera, ProjectUnprojectTest) { TestProjectUnproject(cam_.get(), pX, pY); }
TEST_F(testSimpleRadialCamera, UnProjectprojectTest) { TestUnprojectProject(cam_.get(), pX, pY); }
TEST_F(testSimpleRadialCamera, ProjectionJacobianTest) { TestProjectJacobian(cam_.get(), pX, pY); }
