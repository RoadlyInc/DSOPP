#include "energy/camera_model/fisheye/atan_camera.hpp"

#include <gtest/gtest.h>
#include <Eigen/Eigen>
#include <memory>
#include <random>

#include "generic_camera_test.hpp"

using AtanCamera = dsopp::energy::model::AtanCamera;
using dsopp::operator"" _p;

class testAtanCamera : public ::testing::Test {
 public:
  void SetUp() {
    img_size = Eigen::Vector2<Precision>(3040, 3040);
    focals = Eigen::Vector2<Precision>(855.568, 855.839);
    Eigen::Vector2<Precision> center = Eigen::Vector2<Precision>(1417.72, 1439.66);

    Eigen::VectorX<Precision> polynomial = Eigen::VectorX<Precision>(8);
    polynomial << 0, 0.0591006_p, 0, -0.0105943_p, 0, -0.00467958_p, 0, 0.000500274_p;

    cam_ = std::make_unique<AtanCamera>(img_size, focals, center, polynomial);

    pX = std::uniform_real_distribution<Precision>(kBorder, img_size(0) - kBorder);
    pY = std::uniform_real_distribution<Precision>(kBorder, img_size(1) - kBorder);
  }

  void TearDown() {}

  std::unique_ptr<AtanCamera> cam_;

  Eigen::Vector2<Precision> focals;
  Eigen::Vector2<Precision> img_size;

  const Precision kBorder = 200;

  std::uniform_real_distribution<Precision> pX;
  std::uniform_real_distribution<Precision> pY;
};

TEST_F(testAtanCamera, TestFocalLengths) { TestFocalsEqual(cam_.get(), focals); }

TEST_F(testAtanCamera, ProjectUnprojectTest) { TestProjectUnproject(cam_.get(), pX, pY); }

TEST_F(testAtanCamera, UnProjectprojectTest) { TestUnprojectProject(cam_.get(), pX, pY); }
