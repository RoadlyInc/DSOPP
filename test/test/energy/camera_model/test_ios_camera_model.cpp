#include "energy/camera_model/pinhole/ios_camera_model.hpp"

#include "generic_camera_test.hpp"

#include <gtest/gtest.h>
#include <Eigen/Eigen>
#include <random>

using IOSCamera = dsopp::energy::model::IOSCamera;
using dsopp::operator"" _p;

class testIOSCamera : public ::testing::Test {
 public:
  void SetUp() {
    const Precision kBorderSize = IOSCamera::kBorderSize;
    img_size = Eigen::Vector2<Precision>(1280, 720);
    focals = Eigen::Vector2<Precision>(509.022278, 509.022278);
    Eigen::Vector2<Precision> center(640, 360);
    Eigen::VectorX<Precision> lookup(42);
    lookup << 0.0075767_p, 0.0076070_p, 0.0078076_p, 0.0075842_p, 0.0068205_p, 0.0050126_p, 0.0044590_p, 0.0046658_p,
        0.0033911_p, 0.0025218_p, -0.0002336_p, 0.0005603_p, -0.0003918_p, -0.0009656_p, 0.0002100_p, -0.0006224_p,
        0.0008496_p, 0.0006281_p, 0.0018274_p, 0.0023435_p, 0.0017821_p, 0.0032981_p, 0.0014455_p, 0.0022972_p,
        0.0010508_p, 0.0004276_p, 0.0009804_p, -0.0007222_p, 0.0003875_p, -0.0007304_p, 0.0008617_p, 0.0014343_p,
        0.0033991_p, 0.0053042_p, 0.0073864_p, 0.0105506_p, 0.0139550_p, 0.0171862_p, 0.0213167_p, 0.0242997_p,
        0.0267861_p, 0.0288220_p;

    cam_ = std::make_unique<IOSCamera>(img_size, focals, center, lookup);

    pX = std::uniform_real_distribution<Precision>(kBorderSize, img_size(0) - 1 - kBorderSize);
    pY = std::uniform_real_distribution<Precision>(kBorderSize, img_size(1) - 1 - kBorderSize);
  }
  void TearDown() {}

  std::unique_ptr<IOSCamera> cam_;
  Eigen::Vector2<Precision> focals;
  Eigen::Vector2<Precision> img_size;

  std::uniform_real_distribution<Precision> pX;
  std::uniform_real_distribution<Precision> pY;
};

TEST_F(testIOSCamera, TestFocalLengths) { TestFocalsEqual(cam_.get(), focals); }

TEST_F(testIOSCamera, ProjectUnprojectTest) { TestProjectUnproject(cam_.get(), pX, pY); }

TEST_F(testIOSCamera, UnProjectprojectTest) { TestUnprojectProject(cam_.get(), pX, pY); }
