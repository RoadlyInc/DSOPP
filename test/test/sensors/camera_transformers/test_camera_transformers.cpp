#include "sensors/camera/camera.hpp"

#include <glog/logging.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <opencv2/core.hpp>

#include "common/settings.hpp"
#include "common/time/time.hpp"
#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/camera_model/pinhole/simple_radial.hpp"
#include "features/camera/camera_features.hpp"
#include "features/camera/frame_embedding_extractor.hpp"
#include "features/camera/sobel_tracking_features_extractor.hpp"
#include "mock_camera_provider.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"
#include "sensors/camera_calibration/fabric.hpp"
#include "sensors/camera_calibration/mask/camera_mask.hpp"

#include "sensor/data_frame.hpp"
#include "sensor/synchronized_frame.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"
#include "sensors/camera_providers/camera_data_frame.hpp"
#include "sensors/camera_providers/image_folder_provider.hpp"
#include "sensors/camera_transformers/camera_resizer.hpp"
#include "sensors/camera_transformers/camera_transformer.hpp"

using namespace dsopp::sensors;
using namespace dsopp::sensors::camera_transformers;
using dsopp::Precision;
using dsopp::operator"" _p;

void checkResize(cv::Mat &input, cv::Mat &output, Precision resize_ratio) {
  EXPECT_TRUE(input.type() == output.type());
  int new_width = static_cast<int>(static_cast<Precision>(input.cols) * resize_ratio);
  int new_height = static_cast<int>(static_cast<Precision>(input.rows) * resize_ratio);

  EXPECT_TRUE(new_width == output.cols);
  EXPECT_TRUE(new_height == output.rows);
}

void checkResizeTransformImage(cv::Mat &image, const CameraResizer &resizer, Precision resize_ratio) {
  cv::Mat input = image.clone();
  resizer.transformImage(image);
  checkResize(input, image, resize_ratio);
}

void checkResizeTransformMask(cv::Mat &image, const CameraResizer &resizer, Precision resize_ratio) {
  cv::Mat input = image.clone();
  resizer.transformMask(image);
  checkResize(input, image, resize_ratio);
}

TEST(testCameraTransformers, resizeImageTest) {
  std::vector<Precision> resize_ratios = {1.33, 1., 0.7, 0.5, 0.3};

  int rows_mask = 756;
  int cols_mask = 1235;

  int rows_image = 1000;
  int cols_image = 700;

  cv::Mat mask = cv::Mat::zeros(rows_mask, cols_mask, CV_8UC1);
  cv::Mat image = cv::Mat::zeros(rows_image, cols_image, CV_8UC3);

  for (dsopp::Precision resize_ratio : resize_ratios) {
    CameraResizer resizer(resize_ratio);
    checkResizeTransformImage(image, resizer, resize_ratio);
    checkResizeTransformMask(mask, resizer, resize_ratio);
  }
}

template <class Model>
void checkResizeCalibration(const CameraResizer &resizer, Precision resize_ratio,
                            std::unique_ptr<calibration::CameraCalibration> &&calibration_ptr) {
  using Vec2 = Eigen::Vector2<Precision>;

  auto initial_camera = calibration_ptr->cameraModel<Model>();
  auto initial_image_size = calibration_ptr->image_size();

  resizer.transformCalibration(*calibration_ptr);
  auto resized_camera = calibration_ptr->cameraModel<Model>();

  Precision step_size = 0.25;
  Vec2 center = initial_image_size / 2.0_p;
  std::vector<Eigen::Vector2<Precision>> points = {
      Vec2(center + Vec2(step_size * center.x(), step_size * center.y())),
      Vec2(center + Vec2(-step_size * center.x(), step_size * center.y())),
      Vec2(center + Vec2(step_size * center.x(), -step_size * center.y())),
      Vec2(center + Vec2(-step_size * center.x(), -step_size * center.y()))};

  for (auto &point : points) {
    Eigen::Vector3<Precision> ray;
    EXPECT_TRUE(initial_camera->unproject(point, ray));
    Eigen::Vector2<Precision> pixel_resized;
    EXPECT_TRUE(resized_camera->project(ray, pixel_resized));

    EXPECT_LE((pixel_resized - resize_ratio * point).eval().norm(), 0.1_p);
  }
}

TEST(testCameraTransformers, resizeCalibrationTest) {
  std::vector<Precision> resize_ratios = {1.33, 1., 0.7, 0.5, 0.3};

  auto focals = Eigen::Vector2<Precision>(448, 448);
  auto img_size = Eigen::Vector2<Precision>(1280, 640);
  Eigen::Vector2<Precision> center(640, 360);

  focals = Eigen::Vector2<Precision>(509.022278, 509.022278);
  Eigen::Vector2<Precision> coeffs(0.022297781517582316, -0.01355120683821621);

  for (dsopp::Precision resize_ratio : resize_ratios) {
    CameraResizer resizer(resize_ratio);
    auto pinhole = std::make_unique<calibration::CameraCalibration>(
        img_size, Eigen::Vector4<Precision>(focals(0), focals(1), center(0), center(1)),
        dsopp::energy::model::ModelType::kPinholeCamera);
    auto radial = std::make_unique<calibration::CameraCalibration>(
        img_size, Eigen::Vector<Precision, 5>(focals(0), center(0), center(1), coeffs(0), coeffs(1)),
        dsopp::energy::model::ModelType::kSimpleRadialCamera);

    checkResizeCalibration<dsopp::energy::model::PinholeCamera<Precision>>(resizer, resize_ratio, std::move(pinhole));
    checkResizeCalibration<dsopp::energy::model::SimpleRadialCamera<Precision>>(resizer, resize_ratio,
                                                                                std::move(radial));
  }
}

TEST(testCameraTransformers, cameraWithTransformsTest) {
  std::vector<Precision> resize_ratios = {1.33, 1., 0.7, 0.5, 0.3};

  int rows_image = 1000;
  int cols_image = 700;

  for (auto resize_ratio : resize_ratios) {
    cv::Mat image(rows_image, cols_image, CV_8UC3, cv::Scalar(255, 255, 255));
    image.at<cv::Vec3b>(50, 50) = cv::Vec3b(0, 0, 0);

    int new_width = static_cast<int>(static_cast<Precision>(image.cols) * resize_ratio);
    int new_height = static_cast<int>(static_cast<Precision>(image.rows) * resize_ratio);

    auto provider = std::make_unique<providers::MockCameraProvider>();

    auto *data_frame = new providers::CameraDataFrame(0, std::move(image), dsopp::time(std::chrono::milliseconds(0)));

    EXPECT_CALL(*provider, nextFrameProxy()).WillOnce(::testing::Return(data_frame));
    auto calib = calibration::CameraCalibration(Eigen::Vector2<Precision>(cols_image, rows_image),
                                                Eigen::Vector4<Precision>::Zero(),
                                                dsopp::energy::model::ModelType::kPinholeCamera);
    auto photo_calib = calibration::photometric_calibration::create(TEST_DATA_DIR "track30seconds/pcalib.txt");
    auto vignetting = cv::Mat();

    Eigen::Vector2<int> im_size = calib.image_size().cast<int>();
    auto camera_mask = calibration::CameraMask(im_size.y(), im_size.x());

    std::vector<std::unique_ptr<camera_transformers::CameraTransformer>> transformers;
    transformers.emplace_back(std::make_unique<CameraResizer>(resize_ratio));

    auto settings = std::make_unique<calibration::CameraSettings>(std::move(calib), std::move(photo_calib),
                                                                  std::move(vignetting), std::move(camera_mask));
    Camera camera("camera_1", 0, *settings, std::move(provider),
                  std::make_unique<dsopp::features::SobelTrackingFeaturesExtractor>(), std::move(transformers));
    SynchronizedFrame frame(camera.nextFrameId(), camera.nextFrameTime());
    camera.processNextDataFrame(frame);

    cv::Mat output = frame.cameraFeatures().at(0)->frameData();

    EXPECT_TRUE(new_width == output.cols) << new_width << " " << output.cols;
    EXPECT_TRUE(new_height == output.rows) << new_height << " " << output.rows;
  }
}
