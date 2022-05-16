
#include <numbers>

#include <glog/logging.h>
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <iomanip>

#include "common/settings.hpp"
#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/camera_model/pinhole/simple_radial.hpp"
#include "energy/n_point_solvers/include/energy/n_point_solvers/pure_rotation_estimator.hpp"
#include "energy/problems/photometric_bundle_adjustment/ceres_photometric_bundle_adjustment.hpp"
#include "feature_based_slam/features/distinct_features_extractor_orb.hpp"
#include "feature_based_slam/initialization_strategy/wait_for_movement_keyframe_strategy.hpp"
#include "feature_based_slam/tracker/monocular_initializer.hpp"
#include "feature_based_slam/tracker/monocular_tracker.hpp"
#include "features/camera/frame_embedding_extractor.hpp"
#include "features/camera/pattern_patch.hpp"
#include "features/camera/sobel_tracking_features_extractor.hpp"
#include "features/camera/tracking_feature.hpp"
#include "ransac/random_sequence_generator.hpp"
#include "sensors/camera/camera.hpp"
#include "sensors/camera_calibration/fabric.hpp"
#include "test/tools/depth_gt.hpp"

#include "sensor/synchronized_frame.hpp"
#include "sensors/camera_providers/image_folder_provider.hpp"
#include "sensors/camera_providers/image_video_provider.hpp"
#include "test/tools/tum_gt.hpp"
#include "tracker/landmarks_activator/landmarks_activator.hpp"

using Precision = dsopp::Precision;

std::pair<Precision, Precision> angleAndDistanceDifference(const Sophus::SE3<Precision> &T1,
                                                           const Sophus::SE3<Precision> &T2) {
  Sophus::SE3 diff = T1 * T2.inverse();

  const Precision dist =
      acos(T1.translation().normalized().dot(T2.translation().normalized())) * 180 / std::numbers::pi_v<Precision>;
  const Precision angle = diff.so3().log().norm() * 180 / std::numbers::pi_v<Precision>;

  return {angle, dist};
}

void checkPosesEquality(const Sophus::SE3<Precision> &T1, const Sophus::SE3<Precision> &T2, Precision maxDist,
                        Precision maxAngle) {
  auto [angle, dist] = angleAndDistanceDifference(T1, T2);
  LOG(INFO) << dist << " " << angle;
  EXPECT_LE(angle, maxAngle) << "Angle: " << (angle) << " > " << (maxAngle);
  EXPECT_LE(dist, maxDist) << "Translation angle: " << (dist) << " > " << (maxDist);
}

namespace dsopp::feature_based_slam::tracker {
std::tuple<std::unique_ptr<Tracker>, std::unique_ptr<sensors::Camera>,
           std::unique_ptr<sensors::calibration::CameraSettings>>
createTrackerCameraAndSettings(size_t start_frame) {
  using Calibration = sensors::calibration::CameraCalibration;
  using Model = typename energy::model::PinholeCamera<Precision>;
  using FbsTracker = typename feature_based_slam::tracker::MonocularTracker<Model>;
  using InitializerType = MonocularInitializer<Model>;
  Eigen::Vector<Precision, Model::DoF> intrinsics;
  Precision focal = 448.15_p;
  intrinsics << focal, focal, 640, 360;

  auto calibration =
      Calibration(Eigen::Vector2<Precision>(1280, 720), intrinsics, energy::model::ModelType::kPinholeCamera);
  auto model = calibration.cameraModel<Model>();
  auto init_model = calibration.cameraModel<Model>();
  auto photometric_calibration =
      sensors::calibration::photometric_calibration::create(TEST_DATA_DIR "track30seconds/pcalib.txt");
  auto vignetting = sensors::calibration::vignetting::create(TEST_DATA_DIR "track30seconds/vignetting.png");

  auto provider = std::make_unique<sensors::providers::ImageVideoProvider>(
      TEST_DATA_DIR "track30seconds/images.mkv", TEST_DATA_DIR "track30seconds/times.csv", start_frame);

  // create depth estimation
  auto pattern = std::make_unique<dsopp::features::PatternPatch>();
  Eigen::Vector2<Precision> im_size = calibration.image_size();
  auto camera_mask = sensors::calibration::CameraMask(static_cast<int>(im_size.y()), static_cast<int>(im_size.x()));

  auto settings = std::make_unique<sensors::calibration::CameraSettings>(
      std::move(calibration), std::move(photometric_calibration), std::move(vignetting), std::move(camera_mask));
  auto camera = std::make_unique<sensors::Camera>("camera_1", 0, *settings, std::move(provider),
                                                  std::make_unique<dsopp::features::SobelTrackingFeaturesExtractor>());
  auto keyframe_strategy =
      std::make_unique<feature_based_slam::initialization_strategy::WaitForMovementInitializerKeyframeStrategy>(5, 0.7);
  auto features_extractor = std::make_unique<feature_based_slam::features::DistinctFeaturesExtractorORB>(1000);

  feature_based_slam::tracker::Options options;

  std::function initializer_fabric = [&](feature_based_slam::track::Track &track,
                                         const feature_based_slam::features::DistinctFeaturesExtractor &featurer)
      -> std::unique_ptr<feature_based_slam::tracker::Initializer> {
    return std::make_unique<InitializerType>(track, camera->id(), featurer, std::move(keyframe_strategy), options,
                                             std::move(init_model));
  };
  auto monocular_tracker = std::make_unique<FbsTracker>(camera->id(), std::move(features_extractor), initializer_fabric,
                                                        std::move(model), options);

  return {std::move(monocular_tracker), std::move(camera), std::move(settings)};
}

void evaluateError(size_t startFrame) {
  /** angle between translation directions */
  const Precision kMaxDist = 30;  // in degrees
  const Precision kMaxAngle = 1;  // in degrees

  auto [tracker, camera, settings] = createTrackerCameraAndSettings(startFrame);

  const size_t kNumberOfThreads = 6;
  while (!tracker->initialized()) {
    dsopp::sensors::SynchronizedFrame frame(camera->nextFrameId(), camera->nextFrameTime());
    camera->processNextDataFrame(frame);
    tracker->tick(frame, kNumberOfThreads);
  }
  auto gt = dsopp::test_tools::TumGt(TEST_DATA_DIR "track30seconds/gt.tum");

  const size_t framesN = tracker->evaluatedFramesSize();

  const Sophus::SE3<Precision> &gt_cam0_world = gt.getPose(tracker->timestamp(0)).inverse();

  std::vector<std::pair<Precision, Precision>> errors;
  for (size_t i = 1; i < framesN; ++i) {
    if (!tracker->isInitialized(i)) continue;

    const Sophus::SE3<Precision> &gt_world_cam2 = gt.getPose(tracker->timestamp(i));
    Sophus::SE3<Precision> gt_pose = gt_cam0_world * gt_world_cam2;
    gt_pose.translation().normalize();
    Sophus::SE3<Precision> initialized_pose = tracker->tWorldAgent(i);
    initialized_pose.translation().normalize();
    checkPosesEquality(gt_pose, initialized_pose, kMaxDist, kMaxAngle);
  }
}

TEST(monocular_initializer, monocular_initializer) {
  const size_t kIterationNumber = 1;
  for (auto firstFrame : {0, 50, 300, 460}) {
    for (size_t i = 0; i < kIterationNumber; ++i) {
      evaluateError(static_cast<size_t>(firstFrame));
    }
  }
}

TEST(monocular_initializer, randomSequenceGenerator) {
  const size_t kTestN = 50;
  const size_t kMax = 100;
  const size_t kSequenceSize = 10;
  for (size_t i = 0; i < kTestN; ++i) {
    auto sequence = ransac::generateRandomSequence<kSequenceSize>(kMax);
    for (size_t i1 = 0; i1 < sequence.size(); ++i1) {
      for (size_t i2 = 0; i2 < i1; ++i2) {
        CHECK(sequence[i1] != sequence[i2]);
      }
    }
  }
}

TEST(monocular_initializer, pureRotationSolver) {
  std::mt19937 rng;
  std::uniform_real_distribution<Precision> angle(-0.2_p, 0.2_p);
  std::uniform_real_distribution<Precision> coord(0, 10);
  energy::model::PinholeCamera model(Eigen::Vector2<Precision>(1280, 720), Eigen::Vector2d(448, 448),
                                     Eigen::Vector2d(640, 320));

  const size_t kTestN = 10;
  const int kMinSamples = 3;
  for (size_t iteration = 0; iteration < kTestN; ++iteration) {
    Sophus::SO3<Precision> rotation_t_r =
        Sophus::SO3<Precision>::exp(Eigen::Vector3<Precision>(angle(rng), angle(rng), angle(rng)));
    Eigen::Matrix<Precision, kMinSamples, 2> reference_points, target_points;
    for (int i = 0; i < kMinSamples; ++i) {
      Eigen::Vector3<Precision> reference_bearing(coord(rng), coord(rng), coord(rng));
      Eigen::Vector3<Precision> target_bearing = rotation_t_r * reference_bearing;

      Eigen::Vector2<Precision> reference_point, target_point;
      if (!model.project(reference_bearing, reference_point) || !model.project(target_bearing, target_point)) {
        i--;
        continue;
      }

      reference_points.row(i) = reference_point;
      target_points.row(i) = target_point;
    }

    energy::n_point_solvers::PureRorationSolver<kMinSamples, energy::model::PinholeCamera<double>, Precision> solver(
        model);
    bool converged = solver.solve(reference_points, target_points);

    CHECK(converged);
    Precision err = (solver.solution().inverse() * rotation_t_r).log().norm();
    CHECK_LE(err, 1e-4);
  }
}

TEST(monocular_tracker, monocular_tracker_determinacy) {
  const size_t kTestN = 3;
  const size_t kNumberOfFramesToTrack = 20;

  std::vector<Sophus::SE3<Precision>> initial_poses;

  for (size_t iteration = 0; iteration < kTestN; ++iteration) {
    auto [tracker, camera, settings] = createTrackerCameraAndSettings(0);

    const size_t kNumberOfThreads = 1;
    for (size_t i = 0; i < kNumberOfFramesToTrack; ++i) {
      dsopp::sensors::SynchronizedFrame frame(camera->nextFrameId(), camera->nextFrameTime());
      camera->processNextDataFrame(frame);
      tracker->tick(frame, kNumberOfThreads);
    }

    initial_poses.push_back(tracker->tWorldAgent(tracker->evaluatedFramesSize() - 1));

    EXPECT_TRUE(initial_poses.back().matrix() == initial_poses[0].matrix())
        << (initial_poses.back().matrix() - initial_poses[0].matrix()).norm();
  }
}

TEST(monocular_tracker, monocular_tracker) {
  /** angle between translation directions */
  const Precision kMaxDist = 3;   // in degrees
  const Precision kMaxAngle = 1;  // in degrees

  const size_t kNumberOfThreads = 6;
  const size_t kStartFrameToTrack = 240;
  const size_t kEndFrameToTrack = 340;

  auto [tracker, camera, settings] = createTrackerCameraAndSettings(0);

  for (size_t i = kStartFrameToTrack; i < kEndFrameToTrack; ++i) {
    dsopp::sensors::SynchronizedFrame frame(camera->nextFrameId(), camera->nextFrameTime());
    camera->processNextDataFrame(frame);
    tracker->tick(frame, kNumberOfThreads);
  }
  auto gt = dsopp::test_tools::TumGt(TEST_DATA_DIR "track30seconds/gt.tum");

  const size_t framesN = tracker->evaluatedFramesSize();

  std::vector<Precision> dist_errors, angle_errors;
  for (size_t i = 1; i < framesN; ++i) {
    if (!tracker->isInitialized(i)) continue;

    Sophus::SE3<Precision> gt_pose =
        gt.getPose(tracker->timestamp(i - 1)).inverse() * gt.getPose(tracker->timestamp(i));
    gt_pose.translation().normalize();
    Sophus::SE3<Precision> initialized_pose = tracker->tWorldAgent(i - 1).inverse() * tracker->tWorldAgent(i);
    initialized_pose.translation().normalize();
    auto [angle, dist] = angleAndDistanceDifference(gt_pose, initialized_pose);
    dist_errors.push_back(dist);
    angle_errors.push_back(angle);
  }

  Precision angle = std::accumulate(angle_errors.begin(), angle_errors.end(), 0._p) / Precision(angle_errors.size());
  Precision dist = std::accumulate(dist_errors.begin(), dist_errors.end(), 0._p) / Precision(dist_errors.size());
  LOG(INFO) << dist << " " << angle;
  EXPECT_LE(angle, kMaxAngle) << "Mean angle: " << (angle) << " > " << (kMaxAngle);
  EXPECT_LE(dist, kMaxDist) << "Mean translation angle: " << (dist) << " > " << (kMaxDist);
}

}  // namespace dsopp::feature_based_slam::tracker
