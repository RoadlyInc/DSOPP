#include "sensors/camera/camera.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "common/settings.hpp"
#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "features/camera/frame_embedding_extractor.hpp"
#include "features/camera/sobel_tracking_features_extractor.hpp"
#include "mock_camera_provider.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"
#include "sensors/camera_calibration/fabric.hpp"
#include "sensors/camera_calibration/mask/camera_mask.hpp"

#include "sensor/synchronized_frame.hpp"
#include "sensors/camera_providers/camera_data_frame.hpp"
#include "sensors/camera_providers/image_folder_provider.hpp"

namespace dsopp {
namespace sensors {

TEST(testCamera, oneCamera) {
  cv::Mat image(100, 100, CV_8UC3, cv::Scalar(255, 255, 255));
  image.at<cv::Vec3b>(50, 50) = cv::Vec3b(0, 0, 0);
  auto provider = std::make_unique<providers::MockCameraProvider>();
  auto *data_frame = new providers::CameraDataFrame(0, std::move(image), 1_p, time(std::chrono::milliseconds(0)));
  EXPECT_CALL(*provider, nextFrameProxy()).WillOnce(::testing::Return(data_frame));
  auto calib = calibration::CameraCalibration(Eigen::Vector2<Precision>::Zero(), Eigen::Vector4<Precision>::Zero(),
                                              energy::model::ModelType::kPinholeCamera);
  auto photo_calib = calibration::photometric_calibration::create(TEST_DATA_DIR "track30seconds/pcalib.txt");
  auto vignetting = calibration::vignetting::create(TEST_DATA_DIR "track30seconds/vignetting.png");

  Eigen::Vector2<int> im_size = calib.image_size().cast<int>();
  auto camera_mask = calibration::CameraMask(im_size.y(), im_size.x());

  auto settings = std::make_unique<calibration::CameraSettings>(std::move(calib), std::move(photo_calib),
                                                                std::move(vignetting), std::move(camera_mask));
  Camera camera("camera_1", 0, *settings, std::move(provider),
                std::make_unique<features::SobelTrackingFeaturesExtractor>());

  SynchronizedFrame frame(camera.nextFrameId(), camera.nextFrameTime());
  camera.processNextDataFrame(frame);
}

}  // namespace sensors
}  // namespace dsopp
