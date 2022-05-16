#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include "common/settings.hpp"
#include "common/time/time.hpp"
#include "energy/problems/photometric_bundle_adjustment/ceres_photometric_bundle_adjustment.hpp"
#include "features/camera/camera_features.hpp"
#include "features/camera/ceres_grid.hpp"
#include "features/camera/frame_embedding_extractor.hpp"
#include "features/camera/pattern_patch.hpp"
#include "features/camera/pixel_data_frame.hpp"
#include "features/camera/sobel_tracking_features_extractor.hpp"
#include "features/camera/tracking_feature.hpp"
#include "mock_camera_provider.hpp"
#include "sensors/camera/camera.hpp"
#include "sensors/camera_calibration/fabric.hpp"

#include "sensor/synchronized_frame.hpp"
#include "sensors/camera_providers/image_folder_provider.hpp"
#include "sensors/camera_providers/image_video_provider.hpp"
#include "test/tools/depth_gt.hpp"
#include "test/tools/solver_test_data.hpp"
#include "track/active_odometry_track.hpp"
#include "track/frames/active_keyframe.hpp"
#include "track/landmarks/active_tracking_landmark.hpp"
#include "tracker/build_features.hpp"
#include "tracker/depth_estimators/depth_estimation.hpp"

namespace dsopp {
namespace energy {
namespace problem {
TEST(affine_brightness, affine_brightness) {
  const Precision kGtA = 1;
  const Precision kMaxAGt = 1e-8_p;

  const Precision kGtB = 1e-4_p;
  const Precision kMaxBGt = 1e-3_p;

  const size_t kFrameOffset = 1;
  using Calibration = sensors::calibration::CameraCalibration;
  using Model = energy::model::PinholeCamera<Precision>;
  static constexpr energy::model::ModelType ModelType = energy::model::ModelType::kPinholeCamera;
  using SE3 = energy::motion::SE3<Precision>;

  auto calibration = Calibration(Eigen::Vector2<Precision>(1280, 720),
                                 Eigen::Vector4<Precision>(448.15_p, 448.15_p, 640, 360), ModelType);

  auto provider = std::make_unique<sensors::providers::ImageVideoProvider>(TEST_DATA_DIR "track30seconds/images.mkv",
                                                                           TEST_DATA_DIR "track30seconds/times.csv");

  auto data_frame_1 = provider->nextFrame();
  auto data_frame_2 = provider->nextFrame();

  auto mock_provider = std::make_unique<sensors::providers::MockCameraProvider>();

  cv::Mat image_1 = data_frame_1->data();
  cv::Mat mock_image_1 = data_frame_1->data();
  cv::Mat mock_image_2;
  image_1.convertTo(mock_image_2, CV_8UC1, kGtA, kGtB);

  auto mock_data_frame_1 =
      new sensors::providers::CameraDataFrame(data_frame_1->id(), std::move(mock_image_1), data_frame_1->timestamp());
  auto mock_data_frame_2 =
      new sensors::providers::CameraDataFrame(data_frame_2->id(), std::move(mock_image_2), data_frame_2->timestamp());

  int counter = 0;

  EXPECT_CALL(*mock_provider, nextFrameProxy()).WillRepeatedly([&]() noexcept {
    if (counter == 0) {
      counter++;
      return mock_data_frame_1;
    } else
      return mock_data_frame_2;
  });

  auto model = calibration.cameraModel<Model>();
  size_t sensor = 0;
  auto camera_mask = sensors::calibration::CameraMask(image_1.rows, image_1.cols);
  sensors::calibration::CameraSettings::PhotometricCalibration photometric_calibration;
  std::iota(photometric_calibration.begin(), photometric_calibration.end(), 0);
  auto settings = std::make_unique<sensors::calibration::CameraSettings>(
      std::move(calibration), std::move(photometric_calibration), cv::Mat(), std::move(camera_mask));
  sensors::Camera camera("camera_1", 0, *settings, std::move(mock_provider),
                         std::make_unique<dsopp::features::SobelTrackingFeaturesExtractor>());

  std::vector<std::unique_ptr<sensors::SynchronizedFrame>> frames(kFrameOffset + 1);
  for (size_t i = 0; i <= kFrameOffset; i++) {
    frames[i] = std::make_unique<sensors::SynchronizedFrame>(camera.nextFrameId(), camera.nextFrameTime());
    camera.processNextDataFrame(*frames[i]);
  }
  auto& frame1 = *frames[0]->cameraFeatures().at(sensor);
  auto& frame2 = *frames[kFrameOffset]->cameraFeatures().at(sensor);
  track::ActiveOdometryTrack<SE3> track;

  auto gt_depth_data =
      test_tools::DepthGt(TEST_DATA_DIR "track30seconds/CameraDepth", TEST_DATA_DIR "track30seconds/times.csv");
  auto gt_depth_frame = gt_depth_data.getFrame(frame1.timestamp());
  track.pushFrame(frame1.id(), frame1.timestamp(), Sophus::SE3<Precision>());
  {
    auto pyramids = frame1.movePixelData();
    track.lastKeyframe().pushPyramid(sensor, features::movePyramidAndDelete(pyramids));
    track.lastKeyframe().pushPyramidOfMasks(sensor, frame1.movePyramidOfMasks());
  }
  track.lastKeyframe().pushImmatureLandmarks(sensor, tracker::buildFeatures(frame1.tracking(), *model));

  const size_t landmarkSize = track.activeFrames().back()->immatureLandmarks(sensor).size();
  for (size_t i = 0; i < landmarkSize; ++i) {
    auto& landmark = track.activeFrames().back()->getImmatureLandmark(sensor, i);
    const auto& coords = landmark.projection();
    Precision idepth = 1._p / (*gt_depth_frame)[static_cast<size_t>(coords(0))][static_cast<size_t>(coords(1))];
    landmark.setIdepthMin(idepth);
    landmark.setIdepthMax(idepth);
  }
  const size_t kMaxIteration = 50;
  const Precision kInitialTrustRegionRadius = 1e5;
  const Precision kFunctionTolerance = 1e-8_p;
  const Precision kParameterTolerance = 1e-8_p;
  auto dsopp_solver = CeresPhotometricBundleAdjustment<SE3, Model, Pattern::kSize, features::PixelMap, true, false>(
      energy::problem::TrustRegionPhotometricBundleAdjustmentOptions<double>(
          kMaxIteration, kInitialTrustRegionRadius, kFunctionTolerance, kParameterTolerance,
          Eigen::Vector2<double>::Constant(1e12), 1e16, 5));
  dsopp_solver.pushFrame(*track.activeFrames().back(), 0, *model);
  track::ActiveKeyframe<SE3>::Pyramids nonkeyframe_pyramids;
  {
    auto pyramids = frame2.movePixelData();
    nonkeyframe_pyramids[sensor] = features::movePyramidAndDelete(pyramids);
  }
  dsopp_solver.pushFrame(frame2.timestamp(), Sophus::SE3<Precision>(), nonkeyframe_pyramids,
                         {{sensor, camera.pyramidOfMasks()[0]}}, Eigen::Vector2<Precision>::Zero(), 0, *model);
  dsopp_solver.solve(4);
  auto reference_affine_brightness_dsopp = dsopp_solver.getAffineBrightness(frame1.timestamp());
  auto target_affine_brightness_dsopp = dsopp_solver.getAffineBrightness(frame2.timestamp());
  Precision a_dsopp = std::exp(target_affine_brightness_dsopp[0] - reference_affine_brightness_dsopp[0]);
  Precision b_dsopp = target_affine_brightness_dsopp[1] - a_dsopp * reference_affine_brightness_dsopp[1];
  EXPECT_LE(std::abs(a_dsopp - kGtA), kMaxAGt);
  EXPECT_LE(std::abs(b_dsopp - kGtB), kMaxBGt);

  auto ceres_solver = CeresPhotometricBundleAdjustment<SE3, Model, Pattern::kSize, features::CeresGrid, true, false>(
      energy::problem::TrustRegionPhotometricBundleAdjustmentOptions<double>(
          kMaxIteration, kInitialTrustRegionRadius, kFunctionTolerance, kParameterTolerance,
          Eigen::Vector2<double>::Constant(1e12), 1e16, 5));
  ceres_solver.pushFrame(*track.activeFrames().back(), 0, *model);
  ceres_solver.pushFrame(frame2.timestamp(), Sophus::SE3<Precision>(), nonkeyframe_pyramids,
                         {{sensor, camera.pyramidOfMasks()[0]}}, Eigen::Vector2<Precision>::Zero(), 0, *model);
  ceres_solver.solve(4);
  auto reference_affine_brightness_ceres = ceres_solver.getAffineBrightness(frame1.timestamp());
  auto target_affine_brightness_ceres = ceres_solver.getAffineBrightness(frame2.timestamp());
  Precision a_ceres = std::exp(target_affine_brightness_ceres[0] - reference_affine_brightness_ceres[0]);
  Precision b_ceres = target_affine_brightness_ceres[1] - a_ceres * reference_affine_brightness_ceres[1];
  EXPECT_LE(std::abs(a_ceres - kGtA), kMaxAGt);
  EXPECT_LE(std::abs(b_ceres - kGtB), kMaxBGt);
}
}  // namespace problem
}  // namespace energy
}  // namespace dsopp
