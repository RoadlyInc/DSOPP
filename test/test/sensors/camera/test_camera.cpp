#include "sensors/camera/camera.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>

#include "common/settings.hpp"
#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "features/camera/frame_embedding_extractor.hpp"
#include "features/camera/sobel_tracking_features_extractor.hpp"
#include "mock_camera_provider.hpp"
#include "sensor/synchronized_frame.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"
#include "sensors/camera_calibration/fabric.hpp"
#include "sensors/camera_calibration/mask/camera_mask.hpp"
#include "sensors/camera_providers/camera_data_frame.hpp"
#include "sensors/camera_providers/image_folder_provider.hpp"
#include "sensors/sensors_builder/camera_fabric.hpp"

namespace dsopp {
namespace sensors {

namespace {
void cameraProcess(const std::map<std::string, std::any>& parameters) {
  auto transformers = createCameraTransformer(parameters);
  auto camera_settings = createCameraSettings<true>(parameters, transformers);
  auto camera = createCamera(parameters, "camera_1", 0, std::move(transformers), *camera_settings);
  SynchronizedFrame frame(camera->nextFrameId(), camera->nextFrameTime());
  camera->processNextDataFrame(frame);
}
}  // namespace

TEST(testCamera, oneCamera) {
  auto vignetting = calibration::vignetting::create(TEST_DATA_DIR "track30seconds/vignetting.png");
  auto cols = std::max(vignetting.cols, 100);
  auto rows = std::max(vignetting.rows, 100);
  cv::Mat image(rows, cols, CV_8UC3, cv::Scalar(255, 255, 255));
  image.at<cv::Vec3b>(rows / 2, cols / 2) = cv::Vec3b(0, 0, 0);
  auto provider = std::make_unique<providers::MockCameraProvider>();
  auto* data_frame = new providers::CameraDataFrame(0, std::move(image), time(std::chrono::milliseconds(0)));
  EXPECT_CALL(*provider, nextFrameProxy()).WillOnce(::testing::Return(data_frame));
  auto calib = calibration::CameraCalibration(Eigen::Vector2<Precision>::Zero(), Eigen::Vector4<Precision>::Zero(),
                                              energy::model::ModelType::kPinholeCamera);
  auto photo_calib = calibration::photometric_calibration::create(TEST_DATA_DIR "track30seconds/pcalib.txt");

  Eigen::Vector2<int> im_size = calib.image_size().cast<int>();
  auto camera_mask = calibration::CameraMask(im_size.y(), im_size.x());

  auto settings = std::make_unique<calibration::CameraSettings>(std::move(calib), std::move(photo_calib),
                                                                std::move(vignetting), std::move(camera_mask));
  Camera camera("camera_1", 0, *settings, std::move(provider),
                std::make_unique<features::SobelTrackingFeaturesExtractor>());

  SynchronizedFrame frame(camera.nextFrameId(), camera.nextFrameTime());
  camera.processNextDataFrame(frame);
}

TEST(testCamera, CameraFabric) {
  std::map<std::string, std::any> video_provider = {
      {"type", std::string("video")},
      {"video_file", std::string(TEST_DATA_DIR "track_rolling_shutter/global_shutter.mkv")},
      {"timestamps", std::string(TEST_DATA_DIR "track_rolling_shutter/times.csv")}};

  std::map<std::string, std::any> model = {
      {"calibration", std::string(TEST_DATA_DIR "track_rolling_shutter/calib.txt")},
      {"photometric_calibration", std::string(TEST_DATA_DIR "track_rolling_shutter/pcalib.txt")},
      {"vignetting", std::string(TEST_DATA_DIR "track_rolling_shutter/vignetting.png")}};

  std::map<std::string, std::any> parameters;
  parameters.emplace(std::make_pair("provider", video_provider));
  parameters.emplace(std::make_pair("model", model));
  parameters.emplace(std::make_pair("transformations", std::map<std::string, std::any>()));

  cameraProcess(parameters);

  std::string calib_file = TEST_DATA_DIR "calib_simple_radial.txt";
  model["calibration"] = calib_file;
  parameters["model"] = model;
  std::ofstream fout(calib_file);
  fout << "simple_radial\n1920.000000 1080.000000\n730.882724\n960.000000 540.000000\n0.008271\n-0.005140" << std::endl;

  cameraProcess(parameters);

  std::filesystem::remove_all(calib_file);
}

}  // namespace sensors
}  // namespace dsopp
