#include "sensors/camera/camera.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "common/settings.hpp"
#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/camera_model/pinhole/simple_radial.hpp"
#include "features/camera/frame_embedding_extractor.hpp"
#include "features/camera/sobel_tracking_features_extractor.hpp"
#include "mock_camera_provider.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"
#include "sensors/camera_calibration/camera_settings.hpp"
#include "sensors/camera_calibration/fabric.hpp"
#include "sensors/camera_calibration/mask/camera_mask.hpp"

#include "sensor/synchronized_frame.hpp"
#include "sensors/camera_providers/camera_data_frame.hpp"
#include "sensors/camera_providers/image_folder_provider.hpp"

namespace dsopp {
namespace sensors {

namespace {
bool imageEqual(const cv::Mat &a, const cv::Mat &b) {
  cv::Mat diff;
  cv::absdiff(a, b, diff);
  cv::Mat diff_single_channel = diff.reshape(1, 0);
  cv::Scalar mean = cv::mean(diff_single_channel);
  return mean.val[0] == 0;
}

template <energy::model::Model Model>
void testCameraSettings(calibration::CameraCalibration &&calib) {
  /** Creating Camera */
  Eigen::Vector2<int> im_size = calib.image_size().cast<int>();

  cv::Mat image(im_size.x(), im_size.y(), CV_8UC3, cv::Scalar(255, 255, 255));
  image.at<cv::Vec3b>(50, 50) = cv::Vec3b(0, 0, 0);
  auto provider = std::make_unique<providers::MockCameraProvider>();
  auto photo_calib = calibration::photometric_calibration::create(TEST_DATA_DIR "track30seconds/pcalib.txt");
  auto vignetting = calibration::vignetting::create(TEST_DATA_DIR "track30seconds/vignetting.png");

  auto camera_mask = calibration::CameraMask(im_size.y(), im_size.x());

  auto settings = std::make_unique<calibration::CameraSettings>(std::move(calib), std::move(photo_calib),
                                                                std::move(vignetting), std::move(camera_mask));
  Camera camera("camera_1", 0, *settings, std::move(provider),
                std::make_unique<features::SobelTrackingFeaturesExtractor>());

  /** Start of tests */
  auto &camera_settings = camera.settings();
  auto camera_parameters = camera.settings().calibration().cameraModel<Model>()->intrinsicsParameters();

  /** Check camera parameters equality */
  EXPECT_EQ(camera_settings.calibration().cameraIntrinsics().size(), camera_parameters.size());
  for (int i = 0; i < camera_settings.calibration().cameraIntrinsics().size(); ++i) {
    EXPECT_EQ(camera_settings.calibration().cameraIntrinsics()(i), camera_parameters(i));
  }

  auto proto = camera_settings.proto();
  auto camera_settings_from_proto = calibration::CameraSettings(proto);

  /** Check camera intrinsics equality after proto write-read */
  EXPECT_EQ(camera_settings.calibration().cameraIntrinsics().size(),
            camera_settings_from_proto.calibration().cameraIntrinsics().size());
  for (int i = 0; i < camera_settings.calibration().cameraIntrinsics().size(); ++i) {
    EXPECT_EQ(camera_settings.calibration().cameraIntrinsics()(i),
              camera_settings_from_proto.calibration().cameraIntrinsics()(i));
  }

  /** Check photometric calibration equality */
  for (size_t i = 0; i < camera_settings.photometricCalibration().size(); ++i) {
    EXPECT_EQ(camera_settings.photometricCalibration()[i], camera_settings_from_proto.photometricCalibration()[i]);
  }

  EXPECT_TRUE(imageEqual(camera_settings.vignetting(), camera_settings_from_proto.vignetting()));

  EXPECT_TRUE(imageEqual(camera_settings.cameraMask().data(), camera_settings_from_proto.cameraMask().data()));
}
}  // namespace

TEST(testCameraSettings, protoExporterPinhole) {
  auto calib =
      calibration::CameraCalibration(Eigen::Vector2<Precision>(200, 200), Eigen::Vector4<Precision>(100, 100, 200, 200),
                                     energy::model::ModelType::kPinholeCamera);

  testCameraSettings<energy::model::PinholeCamera<Precision>>(std::move(calib));
}

TEST(testCameraSettings, protoExporterSimpleRaidal) {
  auto calib = calibration::CameraCalibration(Eigen::Vector2<Precision>(200, 200),
                                              Eigen::Vector<Precision, 5>(20, 100, 100, 1, 2),
                                              energy::model::ModelType::kSimpleRadialCamera);

  testCameraSettings<energy::model::SimpleRadialCamera<Precision>>(std::move(calib));
}

}  // namespace sensors
}  // namespace dsopp
