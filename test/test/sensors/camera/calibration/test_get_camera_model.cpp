#include <gtest/gtest.h>

#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"

#include "common/settings.hpp"

namespace dsopp {
namespace sensors {
namespace calibration {
class testPinholeCameraCalibration : public ::testing::Test {
 protected:
  using Calibration = sensors::calibration::CameraCalibration;
  using Pinhole = energy::model::PinholeCamera<Precision>;
  static constexpr energy::model::ModelType PinholeType = energy::model::ModelType::kPinholeCamera;
  void SetUp() override {
    img_size = Eigen::Vector2<Precision>(1280, 720);
    intrinsics = Eigen::Vector4<Precision>(448.15_p, 448.15_p, 640, 360);
    focals = intrinsics.head<2>();
    center = intrinsics.tail<2>();
  }
  Eigen::Vector4<Precision> intrinsics;
  Eigen::Vector2<Precision> img_size;
  Eigen::Vector2<Precision> focals;
  Eigen::Vector2<Precision> center;
};

TEST_F(testPinholeCameraCalibration, get_camera_model) {
  const Precision kEpsilon = 1e-10_p;

  auto calibration = std::make_unique<Calibration>(img_size, intrinsics, PinholeType);
  calibration->crop();
  for (size_t lvl = 0; lvl < Calibration::kNumberOfPyramidLevels; lvl++) {
    auto model = calibration->cameraModel<Pinhole>(lvl);
    ASSERT_LT((model->image_size() - img_size / (1 << lvl)).norm(), kEpsilon);
    ASSERT_LT((model->focal_lengths() - focals / (1 << lvl)).norm(), kEpsilon);
    ASSERT_LT((model->principal_point() - center / (1 << lvl)).norm(), kEpsilon);
  }
}

TEST_F(testPinholeCameraCalibration, cut_off_image) {
  for (size_t i = 0; i < (1 << Calibration::kNumberOfPyramidLevels); i++) {
    img_size += Eigen::Vector2<Precision>::Constant(static_cast<Precision>(i));
    auto calibration = std::make_unique<Calibration>(img_size, intrinsics, PinholeType);
    calibration->crop();
    auto model = calibration->cameraModel<Pinhole>(0);
    ASSERT_TRUE(static_cast<int>(model->image_size()(0)) % (1 << Calibration::kNumberOfPyramidLevels) == 0 and
                static_cast<int>(model->image_size()(1)) % (1 << Calibration::kNumberOfPyramidLevels) == 0);
    ASSERT_TRUE(model->image_size()(0) <= img_size(0) and model->image_size()(1) <= img_size(1));
  }
}

TEST_F(testPinholeCameraCalibration, check_camera_model) {
  const Precision kEpsilon = 1e-10_p;

  auto calibration = std::make_unique<Calibration>(img_size, intrinsics, PinholeType);
  auto base_model = calibration->cameraModel<Pinhole>();
  auto point = Eigen::Vector2<Precision>(20, 50);
  Eigen::Vector3<Precision> point_3d;
  Eigen::Vector2<Precision> point_2d;
  base_model->unproject(point, point_3d);
  for (size_t lvl = 0; lvl < Calibration::kNumberOfPyramidLevels; lvl++) {
    auto model = calibration->cameraModel<Pinhole>(lvl);
    model->project(point_3d, point_2d);
    ASSERT_LT((point - point_2d * (1 << lvl)).norm(), kEpsilon);
  }
}
}  // namespace calibration
}  // namespace sensors
}  // namespace dsopp
