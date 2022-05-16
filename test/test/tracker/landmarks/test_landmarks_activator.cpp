
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include "agent/agent.hpp"
#include "common/settings.hpp"
#include "energy/projector/camera_reproject.hpp"
#include "features/camera/camera_features.hpp"
#include "features/camera/frame_embedding_extractor.hpp"
#include "features/camera/pattern_patch.hpp"
#include "features/camera/pixel_data_frame.hpp"
#include "features/camera/pixel_map.hpp"
#include "features/camera/sobel_tracking_features_extractor.hpp"
#include "features/camera/tracking_feature.hpp"
#include "marginalization/maximum_size_frame_marginalization_strategy.hpp"
#include "mock_camera_provider.hpp"
#include "output_interfaces/track_output_interface.hpp"
#include "sensors/camera/camera.hpp"
#include "sensors/camera_calibration/fabric.hpp"

#include "sensor/synchronized_frame.hpp"
#include "sensors/camera_providers/image_video_provider.hpp"
#include "sensors/sensors.hpp"
#include "test/tools/depth_gt.hpp"
#include "test/tools/solver_test_data.hpp"
#include "test/tools/tum_gt.hpp"
#include "track/active_odometry_track.hpp"
#include "track/active_track.hpp"
#include "track/frames/active_keyframe.hpp"
#include "track/landmarks/active_tracking_landmark.hpp"
#include "track/landmarks/immature_tracking_landmark.hpp"
#include "tracker/build_features.hpp"
#include "tracker/depth_estimators/depth_estimation.hpp"
#include "tracker/landmarks_activator/landmarks_activator.hpp"
#ifdef VISUALIZATION
#include "visualizer/visualizer.hpp"
#endif

#define DEBUG_3D_TRACKING_LANDMARKS_INITIALIZER COMPILE_HIDDEN_CODE

namespace dsopp {
namespace track {
namespace landmarks {

struct LocalImmatureLandmark {
  LocalImmatureLandmark(const ImmatureTrackingLandmark& landmark)
      : is_traced(landmark.isTraced()),
        ready_for_activation(landmark.readyForActivation()),
        projection(landmark.projection()),
        idepth(landmark.idepth()),
        gt_depth(std::numeric_limits<Precision>::infinity()) {}
  bool is_traced;
  bool ready_for_activation;
  Eigen::Vector2<Precision> projection;
  Precision idepth;
  Precision gt_depth;
};

int findImmaturePrototype(const std::vector<LocalImmatureLandmark>& immature_landmarks,
                          const track::landmarks::ActiveTrackingLandmark& active_landmark) {
  const Precision kEps = 1e-10_p;
  for (size_t i = 0; i < immature_landmarks.size(); i++) {
    const auto& immature_landmark = immature_landmarks[i];
    if ((immature_landmark.projection - active_landmark.projection()).norm() < kEps) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

bool checkTheImprovementAfterOptimization(const std::vector<LocalImmatureLandmark>& local_immature_landmarks,
                                          const std::vector<landmarks::ActiveTrackingLandmark>& active_landmarks) {
  const Precision kMaxDepthInCarla = 1000.;

  Precision immature_depth_error = 0;
  Precision active_depth_error = 0;
  for (const auto& landmark : active_landmarks) {
    int idx = findImmaturePrototype(local_immature_landmarks, landmark);
    CHECK(idx >= 0);
    const auto& local_immature_landmark = local_immature_landmarks[static_cast<size_t>(idx)];
    CHECK(local_immature_landmark.is_traced && local_immature_landmark.ready_for_activation);
    if (local_immature_landmark.gt_depth < kMaxDepthInCarla) {
      immature_depth_error += std::abs(local_immature_landmark.idepth * local_immature_landmark.gt_depth - 1);
      active_depth_error += std::abs(landmark.idepth() * local_immature_landmark.gt_depth - 1);
    }
  }
  return active_depth_error < immature_depth_error;
}

TEST(landmarks_activator, landmarks_activator) {
  const Precision kImprovementRation = 0.75;
  const size_t kNumberOfFramesToEstimateDepths = 10;
  const size_t kNumberOfKeyrames = 50;

  using SE3 = energy::motion::SE3<Precision>;

  size_t current_frame_id = 0;
  using Calibration = sensors::calibration::CameraCalibration;
  using Model = energy::model::PinholeCamera<Precision>;
  static constexpr energy::model::ModelType ModelType = energy::model::ModelType::kPinholeCamera;

  auto calibration = Calibration(Eigen::Vector2<Precision>(1280, 720),
                                 Eigen::Vector4<Precision>(448.155164329_p, 448.155164329_p, 640, 360), ModelType);
  auto photometric_calibration =
      sensors::calibration::photometric_calibration::create(TEST_DATA_DIR "track30seconds/pcalib.txt");
  auto vignetting = sensors::calibration::vignetting::create(TEST_DATA_DIR "track30seconds/vignetting.png");
  auto provider = std::make_unique<sensors::providers::ImageVideoProvider>(TEST_DATA_DIR "track30seconds/images.mkv",
                                                                           TEST_DATA_DIR "track30seconds/times.csv");
  auto model = calibration.cameraModel<Model>();

  size_t sensor = 0;
  Eigen::Vector2<int> im_size = calibration.image_size().cast<int>();
  auto camera_mask = sensors::calibration::CameraMask(im_size.y(), im_size.x());

  auto settings = std::make_unique<sensors::calibration::CameraSettings>(
      std::move(calibration), std::move(photometric_calibration), std::move(vignetting), std::move(camera_mask));
  sensors::Camera camera("camera_1", 0, *settings, std::move(provider),
                         std::make_unique<features::SobelTrackingFeaturesExtractor>());

  auto gt = test_tools::TumGt(TEST_DATA_DIR "track30seconds/gt.tum");
  auto gt_depth_data =
      test_tools::DepthGt(TEST_DATA_DIR "track30seconds/CameraDepth", TEST_DATA_DIR "track30seconds/times.csv");

  track::ActiveTrack<SE3> track;
  auto& odometry_track = track.odometryTrack();

  auto landmarks_activator =
      std::make_unique<tracker::LandmarksActivator<SE3, Model, dsopp::features::PixelMap, 1, true>>(
          camera.calibration());

  auto marginalizer = std::make_unique<marginalization::MaximumSizeFrameMarginalizationStrategy<SE3>>(8);

  Precision improved = 0;
  Precision worsened = 0;

  for (size_t k = 0; k < kNumberOfKeyrames; k++) {
    SE3 gt_world_cam1;
    {
      sensors::SynchronizedFrame synchronized_frame(camera.nextFrameId(), camera.nextFrameTime());
      camera.processNextDataFrame(synchronized_frame);
      auto& features = *synchronized_frame.cameraFeatures().at(sensor);

      gt_world_cam1 = gt.getPose(features.timestamp());
      odometry_track.pushFrame(current_frame_id++, features.timestamp(), gt_world_cam1);
      {
        auto pyramid = features.movePixelData();
        odometry_track.lastKeyframe().pushPyramid(sensor, features::movePyramidAndDelete(pyramid));
        odometry_track.lastKeyframe().pushPyramidOfMasks(sensor, features.movePyramidOfMasks());
      }
      odometry_track.lastKeyframe().pushImmatureLandmarks(sensor, tracker::buildFeatures(features.tracking(), *model));
    }

    const size_t immature_landmarks_size = odometry_track.lastKeyframe().immatureLandmarks(sensor).size();

    for (size_t i = 0; i < kNumberOfFramesToEstimateDepths; i++) {
      sensors::SynchronizedFrame synchronized_frame(camera.nextFrameId(), camera.nextFrameTime());
      camera.processNextDataFrame(synchronized_frame);
      auto& features = *synchronized_frame.cameraFeatures().at(sensor);

      const auto& gt_world_agent = gt.getPose(features.timestamp());
      auto gt_cam1_agent = odometry_track.lastKeyframe().tWorldAgent().inverse() * gt_world_agent;
      odometry_track.lastKeyframe().attachTrackingFrame(current_frame_id++, features.timestamp(), gt_cam1_agent);
      const auto& target_grid = features.pixelData().getLevel(0);

      std::vector<std::reference_wrapper<track::landmarks::ImmatureTrackingLandmark>> landmarks;

      for (size_t landmark_i = 0; landmark_i < immature_landmarks_size; ++landmark_i) {
        auto& landmark = odometry_track.lastKeyframe().getImmatureLandmark(sensor, landmark_i);
        landmarks.push_back(landmark);
      }

      tracker::DepthEstimation::estimate<SE3, features::PixelMap, Model, 1>(
          target_grid, landmarks, gt_cam1_agent.inverse(), Eigen::Vector2<Precision>::Zero(),
          Eigen::Vector2<Precision>::Zero(), camera.calibration(), camera.pyramidOfMasks()[0], 9);
    }

    std::vector<LocalImmatureLandmark> local_immature_landmarks;
    if (odometry_track.activeFrames().size() > 1) {
      auto gt_depth_frame =
          gt_depth_data.getFrame(odometry_track.activeFrames()[odometry_track.activeFrames().size() - 2]->timestamp());
      for (const auto& landmark :
           odometry_track.activeFrames()[odometry_track.activeFrames().size() - 2]->immatureLandmarks(sensor)) {
        local_immature_landmarks.emplace_back(LocalImmatureLandmark(landmark));
        local_immature_landmarks.back().gt_depth = (*gt_depth_frame)[static_cast<size_t>(landmark.projection().x())]
                                                                    [static_cast<size_t>(landmark.projection().y())];
      }
    }

    landmarks_activator->activate(odometry_track);

    if (odometry_track.activeFrames().size() > 1) {
      if (checkTheImprovementAfterOptimization(
              local_immature_landmarks,
              odometry_track.activeFrames()[odometry_track.activeFrames().size() - 2]->activeLandmarks(sensor))) {
        improved++;
      } else {
        worsened++;
      }
    }

    marginalizer->marginalize(odometry_track);
  }

  ASSERT_GE(improved / (improved + worsened), kImprovementRation);

#if DEBUG_3D_TRACKING_LANDMARKS_INITIALIZER
#ifdef VISUALIZATION
  // activate landmarks to visualize
  SE3 first_pose = odometry_track.activeFrames().front()->tWorldAgent();
  for (size_t i = 0; i < odometry_track.activeFrames().size(); i++) {
    auto& keyframe = odometry_track.getActiveKeyframe(i);
    keyframe.setTWorldAgent(first_pose.inverse() * keyframe.tWorldAgent());
    for (size_t j = 0; j < keyframe.activeLandmarks(sensor).size(); j++) {
      auto& landmark = keyframe.getActiveLandmark(sensor, j);
      landmark.setRelativeBaseline(1);
    }
  }

  auto visualizer = output::Visualizer(1920, 1080);
  auto* visualizer_FPS = visualizer.createTrackOutputInterface<dsopp::track::ActiveOdometryTrack, SE3>();
  visualizer_FPS->notify(track);
  visualizer.init();
  while (visualizer.running()) {
    visualizer.render();
  }
#endif
#endif
}
}  // namespace landmarks
}  // namespace track
}  // namespace dsopp
