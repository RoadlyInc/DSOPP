#include <gtest/gtest.h>

#include "common/file_tools/camera_frame_times.hpp"
#include "common/settings.hpp"
#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/motion/se3_motion.hpp"
#include "energy/problems/photometric_bundle_adjustment/eigen_photometric_bundle_adjustment.hpp"
#include "features/camera/pixel_data_frame.hpp"
#include "features/camera/sobel_tracking_features_extractor.hpp"
#include "features/camera/tracking_feature.hpp"
#include "mock_camera_provider.hpp"
#include "sensor/synchronized_frame.hpp"
#include "sensors/camera/camera.hpp"
#include "sensors/camera_calibration/fabric.hpp"
#include "sensors/camera_providers/image_video_provider.hpp"
#include "test/tools/depth_gt.hpp"
#include "track/active_odometry_track.hpp"
#include "track/active_track.hpp"
#include "track/connections/connections_container.hpp"
#include "track/connections/frame_connection.hpp"
#include "track/landmarks/active_tracking_landmark.hpp"
#include "track/landmarks/immature_tracking_landmark.hpp"
#include "tracker/build_features.hpp"
#include "tracker/landmarks_activator/landmarks_activator.hpp"

namespace dsopp::energy::problem {
void testAffineBrightness(bool regularize_affine_brightness, Precision exposure_time = 1) {
  const Precision kGtA = 1.1_p;
  const Precision kGtB = 10_p;

  const size_t kSensor = 0;
  const size_t kStartFrame = 10;
  const size_t kMaxPointsPerFrame = 800;

  using Motion = energy::motion::SE3<Precision>;
  using Calibration = sensors::calibration::CameraCalibration;
  using Model = energy::model::PinholeCamera<Precision>;
  constexpr energy::model::ModelType ModelType = energy::model::ModelType::kPinholeCamera;

  srand(0);

  auto calibration = Calibration(Eigen::Vector2<Precision>(1280, 720),
                                 Eigen::Vector4<Precision>(448.155164329_p, 448.155164329_p, 640, 360), ModelType);
  auto photometric_calibration =
      sensors::calibration::photometric_calibration::create(TEST_DATA_DIR "track30seconds/pcalib.txt");
  auto vignetting = sensors::calibration::vignetting::create(TEST_DATA_DIR "track30seconds/vignetting.png");

  auto provider = std::make_unique<sensors::providers::ImageVideoProvider>(
      TEST_DATA_DIR "track30seconds/images.mkv", TEST_DATA_DIR "track30seconds/times.csv", kStartFrame);

  auto data_frame_1 = provider->nextFrame();
  auto data_frame_2 = provider->nextFrame();

  auto mock_provider = std::make_unique<sensors::providers::MockCameraProvider>();

  cv::Mat image_1 = data_frame_1->data();
  cv::Mat mock_image_1 = data_frame_1->data();
  cv::Mat mock_image_2;
  image_1.convertTo(mock_image_2, CV_8UC1, kGtA, kGtB);

  auto mock_data_frame_1 = new sensors::providers::CameraDataFrame(data_frame_1->id(), std::move(mock_image_1), 1,
                                                                   data_frame_1->timestamp());
  auto mock_data_frame_2 = new sensors::providers::CameraDataFrame(data_frame_2->id(), std::move(mock_image_2),
                                                                   exposure_time, data_frame_2->timestamp());

  int counter = 0;

  EXPECT_CALL(*mock_provider, nextFrameProxy()).WillRepeatedly([&]() noexcept {
    if (counter == 0) {
      counter++;
      return mock_data_frame_1;
    } else
      return mock_data_frame_2;
  });

  Eigen::Vector2<int> im_size = calibration.image_size().cast<int>();
  auto camera_mask = sensors::calibration::CameraMask(im_size.y(), im_size.x());
  auto model = calibration.cameraModel<energy::model::PinholeCamera<Precision>>();
  auto settings = std::make_unique<sensors::calibration::CameraSettings>(
      std::move(calibration), std::move(photometric_calibration), std::move(vignetting), std::move(camera_mask));
  auto camera =
      std::make_unique<sensors::Camera>("camera_1", kSensor, *settings, std::move(mock_provider),
                                        std::make_unique<dsopp::features::SobelTrackingFeaturesExtractor>(2000));

  auto gt_depth_data =
      test_tools::DepthGt(TEST_DATA_DIR "track30seconds/CameraDepth", TEST_DATA_DIR "track30seconds/times.csv");

  std::vector<std::unique_ptr<sensors::SynchronizedFrame>> frames(2);
  track::ActiveTrack<Motion> track;
  auto& odometry_track = track.odometryTrack();
  auto landmarks_activator =
      std::make_unique<tracker::LandmarksActivator<Motion, Model, features::PixelMap, 1>>(camera->calibration());
  for (size_t current_frame = 0; current_frame <= 1; ++current_frame) {
    frames[current_frame] =
        std::make_unique<sensors::SynchronizedFrame>(camera->nextFrameId(), camera->nextFrameTime());
    camera->processNextDataFrame(*frames[current_frame]);

    auto features = frames[current_frame]->cameraFeatures().at(kSensor).get();
    auto gt_depth_frame = gt_depth_data.getFrame(features->timestamp());
    odometry_track.pushFrame(features->id(), features->timestamp(), Motion(), features->exposureTime());
    {
      auto pixel_data_frame = features->movePixelData();
      odometry_track.lastKeyframe().pushPyramid(kSensor, features::movePyramidAndDelete(pixel_data_frame));
      odometry_track.lastKeyframe().pushPyramidOfMasks(kSensor, features->movePyramidOfMasks());
    }
    odometry_track.lastKeyframe().pushImmatureLandmarks(kSensor, tracker::buildFeatures(features->tracking(), *model));

    const size_t landmarksSize = odometry_track.lastKeyframe().immatureLandmarks(kSensor).size();
    Precision landmark_pick_probability =
        static_cast<Precision>(kMaxPointsPerFrame) / static_cast<Precision>(landmarksSize);
    for (size_t i = 0; i < landmarksSize; ++i) {
      auto& landmark = odometry_track.lastKeyframe().getImmatureLandmark(kSensor, i);
      const auto& coords = landmark.projection();
      Precision idepth = 1._p / (*gt_depth_frame)[static_cast<size_t>(coords(0))][static_cast<size_t>(coords(1))];
      landmark.setIdepthMin(idepth);
      landmark.setIdepthMax(idepth);
      if (rand() * 1.0 / RAND_MAX <= landmark_pick_probability) {
        landmark.setStatus(track::landmarks::ImmatureStatus::kGood);
      } else {
        landmark.setStatus(track::landmarks::ImmatureStatus::kDelete);
      }
      landmark.setSearchPixelInterval(0);
    }

    landmarks_activator->activate(odometry_track);
  }
  for (size_t k = 0; k < odometry_track.activeFrames().size(); k++) {
    auto& keyframe = odometry_track.getActiveKeyframe(k);
    for (size_t idx = 0; idx < keyframe.activeLandmarks(kSensor).size(); idx++) {
      keyframe.getActiveLandmark(kSensor, idx).setIdepthVariance(2);
      keyframe.getActiveLandmark(kSensor, idx).setRelativeBaseline(1);
    }
  }
  for (auto& reference_frame : odometry_track.activeFrames()) {
    for (auto& target_frame : odometry_track.activeFrames()) {
      if (reference_frame->keyframeId() >= target_frame->keyframeId()) continue;
      auto connection = std::make_unique<track::FrameConnection<typename Motion::Product>>(
          reference_frame->keyframeId(), target_frame->keyframeId());
      connection->addSensorConnection(kSensor, kSensor, reference_frame->activeLandmarks(kSensor).size(),
                                      target_frame->activeLandmarks(kSensor).size());
      reference_frame->addConnection(target_frame->keyframeId(), connection.get());
      target_frame->addConnection(reference_frame->keyframeId(), connection.get());
      odometry_track.connections().add(std::move(connection));
    }
  }

  const size_t kMaxIteration = 50;
  const Precision kInitialTrustRegionRadius = 1e5;
  const Precision kFunctionTolerance = 1e-8_p;
  const Precision kParameterTolerance = 1e-8_p;
  const auto kAffineBrightnessRegularizer = regularize_affine_brightness ? Eigen::Vector2<Precision>::Constant(1e12_p)
                                                                         : Eigen::Vector2<Precision>::Constant(0_p);
  const Precision kFixedStateRegularizer = 1e16_p;
  const auto options = TrustRegionPhotometricBundleAdjustmentOptions<Precision>(
      kMaxIteration, kInitialTrustRegionRadius, kFunctionTolerance, kParameterTolerance, kAffineBrightnessRegularizer,
      kFixedStateRegularizer);

  auto solver = energy::problem::EigenPhotometricBundleAdjustment<Motion, Model>(options);

  const auto& frame1 = *odometry_track.keyframes().front();
  const auto& frame2 = *odometry_track.keyframes().back();
  solver.pushFrame(frame1, 0, *model, FrameParameterization::kFixed);
  solver.pushFrame(frame2, 0, *model, FrameParameterization::kFree);

  solver.solve(1);

  auto target_affine_brightness_dsopp = solver.getAffineBrightness(frame2.timestamp());
  auto a = std::exp(target_affine_brightness_dsopp[0]);
  auto b = target_affine_brightness_dsopp[1];
  if (regularize_affine_brightness) {
    EXPECT_LT(std::abs(a - 1), 1e-3);
    EXPECT_LT(std::abs(b), 1e-3);
  } else {
    EXPECT_GT(exposure_time * a, kGtA + (kGtA - 1) / 2);
    EXPECT_GT(b, kGtB / 3);
    if (exposure_time > 1) {
      EXPECT_LT(a, kGtA + (kGtA - 1) / 2);
    }
  }
}

TEST(affine_brightness, affine_brightness_without_regularization_without_exposure_time) { testAffineBrightness(false); }

TEST(affine_brightness, affine_brightness_with_regularization_without_exposure_time) { testAffineBrightness(true); }

TEST(affine_brightness, affine_brightness_without_regularization_with_exposure_time) {
  testAffineBrightness(false, 10);
}

TEST(affine_brightness, affine_brightness_with_regularization_with_exposure_time) { testAffineBrightness(true, 10); }
}  // namespace dsopp::energy::problem
