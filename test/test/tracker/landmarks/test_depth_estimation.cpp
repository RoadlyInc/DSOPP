
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include "agent/agent.hpp"
#include "common/file_tools/camera_frame_times.hpp"
#include "common/settings.hpp"
#include "energy/epipolar_geometry/epipolar_line.hpp"
#include "energy/epipolar_geometry/epipolar_line_builder.hpp"
#include "energy/projector/camera_reproject.hpp"
#include "features/camera/camera_features.hpp"
#include "features/camera/frame_embedding_extractor.hpp"
#include "features/camera/pattern_patch.hpp"
#include "features/camera/pixel_data_frame.hpp"
#include "features/camera/pixel_map.hpp"
#include "features/camera/sobel_tracking_features_extractor.hpp"
#include "features/camera/tracking_feature.hpp"
#include "mock_camera_provider.hpp"
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

#define DEBUG_2D_TRACKING_LANDMARKS_INITIALIZER COMPILE_HIDDEN_CODE
#define DEBUG_3D_TRACKING_LANDMARKS_INITIALIZER COMPILE_HIDDEN_CODE

#if DEBUG_2D_TRACKING_LANDMARKS_INITIALIZER or DEBUG_3D_TRACKING_LANDMARKS_INITIALIZER

#include "common/image_tools/conversion.hpp"
#include "output_interfaces/track_output_interface.hpp"
#ifdef VISUALIZATION
#include "visualizer/visualizer.hpp"
#endif

#endif

namespace dsopp {
namespace track {
namespace landmarks {

struct LocalImmatureLandmark {
  LocalImmatureLandmark(const ImmatureTrackingLandmark& landmark)
      : status(landmark.status()),
        is_traced(landmark.isTraced()),
        ready_for_activation(landmark.readyForActivation()),
        idepth(landmark.idepth()) {}
  ImmatureStatus status;
  bool is_traced;
  bool ready_for_activation;
  Precision idepth;
  Precision gt_idepth;
};

void updateLocalLandmarks(std::vector<LocalImmatureLandmark>& local_immature_landmarks,
                          const std::vector<landmarks::ImmatureTrackingLandmark>& immature_landmarks) {
  EXPECT_EQ(local_immature_landmarks.size(), immature_landmarks.size());

  for (size_t landmark_i = 0; landmark_i < local_immature_landmarks.size(); ++landmark_i) {
    auto& local_landmark = local_immature_landmarks[landmark_i];
    const auto& landmark = immature_landmarks[landmark_i];

    if (local_landmark.is_traced) {
      EXPECT_TRUE(landmark.isTraced());
    }
    local_landmark.is_traced = landmark.isTraced();

    local_landmark.ready_for_activation = landmark.readyForActivation();

    if (local_landmark.status == ImmatureStatus::kDelete) {
      EXPECT_TRUE(landmark.status() == ImmatureStatus::kDelete);
    }
    if (local_landmark.status == ImmatureStatus::kOutOfBoundary) {
      EXPECT_TRUE(landmark.status() == ImmatureStatus::kOutOfBoundary);
    }
    if (local_landmark.status == ImmatureStatus::kOutlier) {
      EXPECT_TRUE(landmark.status() == ImmatureStatus::kOutlier);
    }
    local_landmark.status = landmark.status();

    local_landmark.idepth = landmark.idepth();
  }
}

template <energy::motion::Motion Motion>
void depthEstimationTestBody(const size_t numberOfKeyframes) {
  const size_t kNumberOfFramesToEstimateDepths = 10;
  const Precision kMinIdepthInCarla = 1._p / 1000._p;

  const Precision kMinRatioOfInliers = 0.65_p;
  const Precision kInlierRelativeErrorTreshold = 0.1_p;
  const Precision kMaxRatioOfOutliers = 0.1_p;
  const Precision kOutlierRelativeErrorTreshold = 2;
  const Precision kMinRatioOfLandmarksToActivate = 0.1_p;

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

  track::ActiveTrack<Motion> track;
  auto& odometry_track = track.odometryTrack();

#if DEBUG_3D_TRACKING_LANDMARKS_INITIALIZER
  auto landmarks_activator =
      std::make_unique<tracker::LandmarksActivator<Motion, Model, dsopp::features::PixelMap, 1, false>>(
          camera.calibration());
#endif

  for (size_t k = 0; k < numberOfKeyframes; k++) {
    Motion gt_world_cam1;
    {
      sensors::SynchronizedFrame synchronized_frame(camera.nextFrameId(), camera.nextFrameTime());
      camera.processNextDataFrame(synchronized_frame);
      auto& features = *synchronized_frame.cameraFeatures().at(sensor);

      gt_world_cam1 = Motion(gt.getPose(features.timestamp()));
      odometry_track.pushFrame(current_frame_id++, features.timestamp(), gt_world_cam1);
      {
        auto pyramid = features.movePixelData();
        odometry_track.lastKeyframe().pushPyramid(sensor, features::movePyramidAndDelete(pyramid));
      }
      odometry_track.lastKeyframe().pushImmatureLandmarks(sensor, tracker::buildFeatures(features.tracking(), *model));
    }

    const size_t immature_landmarks_size = odometry_track.lastKeyframe().immatureLandmarks(sensor).size();

    std::vector<LocalImmatureLandmark> local_immature_landmarks;
    auto gt_depth_frame = gt_depth_data.getFrame(odometry_track.lastKeyframe().timestamp());
    for (const auto& landmark : odometry_track.lastKeyframe().immatureLandmarks(sensor)) {
      local_immature_landmarks.emplace_back(LocalImmatureLandmark(landmark));
      local_immature_landmarks.back().gt_idepth = 1._p /
                                                  (*gt_depth_frame)[static_cast<size_t>(landmark.projection().x())]
                                                                   [static_cast<size_t>(landmark.projection().y())];
    }

    for (size_t i = 0; i < kNumberOfFramesToEstimateDepths; i++) {
      sensors::SynchronizedFrame synchronized_frame(camera.nextFrameId(), camera.nextFrameTime());
      camera.processNextDataFrame(synchronized_frame);
      auto& features = *synchronized_frame.cameraFeatures().at(sensor);

      const auto& gt_world_agent = Motion(gt.getPose(features.timestamp()));
      auto gt_cam1_agent = odometry_track.lastKeyframe().tWorldAgent().inverse() * gt_world_agent;
      odometry_track.lastKeyframe().attachTrackingFrame(current_frame_id++, features.timestamp(), gt_cam1_agent);
      const auto& target_grid = features.pixelData().getLevel(0);
      energy::reprojection::ArrayReprojector<Precision, Model, typename Motion::Product> reprojector(
          *model, gt_cam1_agent.inverse());

      std::vector<std::reference_wrapper<track::landmarks::ImmatureTrackingLandmark>> landmarks;

      for (size_t landmark_i = 0; landmark_i < immature_landmarks_size; ++landmark_i) {
        auto& landmark = odometry_track.lastKeyframe().getImmatureLandmark(sensor, landmark_i);
        landmarks.push_back(landmark);
      }

      tracker::DepthEstimation::estimate<typename Motion::Product, features::PixelMap, Model, 1>(
          target_grid, landmarks, gt_cam1_agent.inverse(), Eigen::Vector2<Precision>::Zero(),
          Eigen::Vector2<Precision>::Zero(), camera.calibration(), camera.pyramidOfMasks()[0], 9);

      updateLocalLandmarks(local_immature_landmarks, odometry_track.lastKeyframe().immatureLandmarks(sensor));
    }

    Precision number_of_inliers = 0;
    Precision number_of_outliers = 0;
    Precision number_of_valid_landmarks = 0;
    for (const auto& local_landmark : local_immature_landmarks) {
      if (local_landmark.is_traced && local_landmark.ready_for_activation &&
          local_landmark.gt_idepth > kMinIdepthInCarla) {
        Precision rel_error = std::abs(local_landmark.idepth - local_landmark.gt_idepth) / local_landmark.gt_idepth;
        if (rel_error < kInlierRelativeErrorTreshold) {
          number_of_inliers++;
        }
        if (rel_error > kOutlierRelativeErrorTreshold) {
          number_of_outliers++;
        }
        number_of_valid_landmarks++;
      }
    }
    if (odometry_track.activeFrames().size() > 2) {
      EXPECT_GT(number_of_inliers / number_of_valid_landmarks, kMinRatioOfInliers) << "Frame: " << current_frame_id;
      EXPECT_LT(number_of_outliers / number_of_valid_landmarks, kMaxRatioOfOutliers) << "Frame: " << current_frame_id;
      EXPECT_GT(number_of_valid_landmarks / static_cast<Precision>(immature_landmarks_size),
                kMinRatioOfLandmarksToActivate)
          << "Frame: " << current_frame_id;
    }

#if DEBUG_3D_TRACKING_LANDMARKS_INITIALIZER
    landmarks_activator->activate(odometry_track);
#endif
  }

#if DEBUG_2D_TRACKING_LANDMARKS_INITIALIZER
  auto& active_frame_1 = odometry_track.activeFrames()[5];
  auto& active_frame_2 = odometry_track.activeFrames()[6];
  const features::PixelMap<1>& reference_grid = active_frame_1->getLevel(sensor, 0);
  const features::PixelMap<1>& target_grid = active_frame_2->getLevel(sensor, 0);
  auto gt_cam2_cam1 = active_frame_2->tWorldAgent().inverse() * active_frame_1->tWorldAgent();

  energy::reprojection::ArrayReprojector<Precision, Model, typename Motion::Product> reprojector(*model, gt_cam2_cam1);

  auto image_reference = common::image_tools::pixelMap2Mat1C(reference_grid);
  auto image_target = common::image_tools::pixelMap2Mat1C(target_grid);
  cv::Mat image;
  cv::hconcat(image_reference, image_target, image);
  long width = reference_grid.width();

  for (const auto& landmark : active_frame_1->immatureLandmarks(sensor)) {
    if (!landmark.isTraced() || !landmark.readyForActivation()) {
      continue;
    }
    auto debug = image.clone();

    Eigen::Vector2<Precision> point2d1 = landmark.projection(), point2d2_min, point2d2_max;
    std::cout << landmark.idepthMin() << " : " << landmark.idepthMax() << std::endl;
    reprojector.reproject(point2d1, landmark.idepthMin(), point2d2_min);
    reprojector.reproject(point2d1, landmark.idepthMax(), point2d2_max);

    cv::circle(debug, cv::Point(static_cast<int>(point2d1.x()), static_cast<int>(point2d1.y())), 5, cv::Scalar(255));
    cv::circle(debug,
               cv::Point(static_cast<int>(point2d2_min.x() + static_cast<Precision>(width)),
                         static_cast<int>(point2d2_min.y())),
               5, cv::Scalar(255));
    cv::circle(debug,
               cv::Point(static_cast<int>(point2d2_max.x() + static_cast<Precision>(width)),
                         static_cast<int>(point2d2_max.y())),
               5, cv::Scalar(255));

    auto epiline_builder =
        energy::epipolar_geometry::EpipolarLineBuilder<Model, typename Motion::Product>(*model, gt_cam2_cam1);

    auto epipolar_line = epiline_builder.build(point2d1);

    for (const auto& p : epipolar_line.points) {
      cv::circle(debug,
                 cv::Point(static_cast<int>(p.projection.x() + static_cast<Precision>(width)),
                           static_cast<int>(p.projection.y())),
                 1, cv::Scalar(255));
    }

    cv::imshow("DEBUG 2D TRACKING LANDMARKS INITIALIZER", debug);
    cv::waitKey();
  }

#endif

#if DEBUG_3D_TRACKING_LANDMARKS_INITIALIZER
#ifdef VISUALIZATION
  // activate landmarks to visualize
  Motion first_pose = odometry_track.activeFrames().front()->tWorldAgent();
  for (size_t i = 0; i < odometry_track.activeFrames().size(); i++) {
    auto& keyframe = odometry_track.getActiveKeyframe(i);
    keyframe.setTWorldAgent(Motion((first_pose.inverse() * keyframe.tWorldAgent()).se3()));
    for (size_t j = 0; j < keyframe.activeLandmarks(sensor).size(); j++) {
      auto& landmark = keyframe.getActiveLandmark(sensor, j);
      landmark.setRelativeBaseline(1);
    }
  }

  auto visualizer = output::Visualizer(1920, 1080);
  auto* visualizer_FPS = visualizer.createTrackOutputInterface<dsopp::track::ActiveOdometryTrack, Motion>();
  visualizer_FPS->notify(track);
  visualizer.init();
  while (visualizer.running()) {
    visualizer.render();
  }
#endif
#endif
}

TEST(depth_estimation, depth_estimation_se3) {
  LOG(INFO) << "SE3 :";
  depthEstimationTestBody<energy::motion::SE3<Precision>>(40);
}

}  // namespace landmarks
}  // namespace track
}  // namespace dsopp
