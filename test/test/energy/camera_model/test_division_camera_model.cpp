#include "energy/camera_model/fisheye/division_model.hpp"

#include "generic_camera_test.hpp"

#include <gtest/gtest.h>
#include <Eigen/Eigen>
#include <random>

using DivisionModelCamera = dsopp::energy::model::DivisionCameraModel<Precision>;
using dsopp::operator"" _p;

class TestDivisionModelCamera : public ::testing::Test {
 public:
  void SetUp() {
    const Precision kBorderSize = DivisionModelCamera::kBorderSize;
    img_size = Eigen::Vector2<Precision>(1280, 720);
    focals = Eigen::Vector2<Precision>(509.022278, 509.022278);
    Eigen::Vector2<Precision> center(640, 360);
    Precision lambda = -3.4_p;

    cam_ = std::make_unique<DivisionModelCamera>(img_size, focals(0), center, lambda);

    pX = std::uniform_real_distribution<Precision>(kBorderSize, img_size(0) - 1 - kBorderSize);
    pY = std::uniform_real_distribution<Precision>(kBorderSize, img_size(1) - 1 - kBorderSize);
  }

  void TearDown() {}

  std::unique_ptr<DivisionModelCamera> cam_;
  Eigen::Vector2<Precision> focals;
  Eigen::Vector2<Precision> img_size;

  std::uniform_real_distribution<Precision> pX;
  std::uniform_real_distribution<Precision> pY;
};

TEST_F(TestDivisionModelCamera, TestFocalLengths) { TestFocalsEqual(cam_.get(), focals); }
TEST_F(TestDivisionModelCamera, ProjectUnprojectTest) { TestProjectUnproject(cam_.get(), pX, pY); }
TEST_F(TestDivisionModelCamera, UnProjectprojectTest) { TestUnprojectProject(cam_.get(), pX, pY); }
