#include "energy/camera_model/fisheye/tum_fov_model.hpp"

#include "generic_camera_test.hpp"

#include <gtest/gtest.h>
#include <Eigen/Eigen>
#include <random>

using TUMFovModel = dsopp::energy::model::TUMFovModel<Precision>;
using dsopp::operator"" _p;

class testTUMFovModel : public ::testing::Test {
 public:
  void SetUp() {
    const Precision kBorderSize = TUMFovModel::kBorderSize;
    img_size = Eigen::Vector2<Precision>(1280, 1024);
    focals = Eigen::Vector2<Precision>(0.349153000000000, 0.436593000000000);
    focals[0] *= img_size[0];
    focals[1] *= img_size[1];
    Eigen::Vector2<Precision> center(0.493140000000000, 0.499021000000000);
    center[0] *= img_size[0];
    center[1] *= img_size[1];

    Precision fov = 0.933271000000000;

    cam_ = std::make_unique<TUMFovModel>(img_size, focals, center, fov);

    Precision eps = 1e-2_p;
    pX = std::uniform_real_distribution<Precision>(kBorderSize + eps, img_size(0) - 1 - kBorderSize - eps);
    pY = std::uniform_real_distribution<Precision>(kBorderSize + eps, img_size(1) - 1 - kBorderSize - eps);
  }

  void TearDown() {}

  std::unique_ptr<TUMFovModel> cam_;
  Eigen::Vector2<Precision> focals;
  Eigen::Vector2<Precision> img_size;

  std::uniform_real_distribution<Precision> pX;
  std::uniform_real_distribution<Precision> pY;
};

TEST_F(testTUMFovModel, TestFocalLengths) { TestFocalsEqual(cam_.get(), focals); }
TEST_F(testTUMFovModel, ProjectUnprojectTest) { TestProjectUnproject(cam_.get(), pX, pY); }
TEST_F(testTUMFovModel, UnProjectprojectTest) { TestUnprojectProject(cam_.get(), pX, pY); }