
#include <numbers>
#include <thread>

#include <glog/logging.h>
#include <Eigen/Dense>

#include "common/file_tools/camera_frame_times.hpp"
#include "common/settings.hpp"
#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/problems/photometric_bundle_adjustment/ceres_photometric_bundle_adjustment.hpp"
#include "feature_based_slam/features/correspondences_finder.hpp"
#include "feature_based_slam/features/distinct_features_extractor.hpp"
#include "feature_based_slam/features/distinct_features_extractor_orb.hpp"
#include "feature_based_slam/initialization_strategy/wait_for_movement_keyframe_strategy.hpp"
#include "feature_based_slam/tracker/monocular_initializer.hpp"
#include "feature_based_slam/tracker/monocular_tracker.hpp"
#include "features/camera/frame_embedding_extractor.hpp"
#include "features/camera/pattern_patch.hpp"
#include "features/camera/sobel_tracking_features_extractor.hpp"
#include "features/camera/tracking_feature.hpp"
#include "sensors/camera/camera.hpp"
#include "sensors/camera_calibration/fabric.hpp"

#include "sensor/synchronized_frame.hpp"
#include "sensors/camera_providers/image_folder_provider.hpp"
#include "sensors/camera_providers/image_video_provider.hpp"
#include "tracker/landmarks_activator/landmarks_activator.hpp"

#include "test/tools/tum_gt.hpp"

using namespace dsopp;
using namespace dsopp::tracker;

std::pair<Precision, Precision> checkPosesEquality(const Sophus::SE3<Precision> &T1, const Sophus::SE3<Precision> &T2) {
  Sophus::SE3 diff = T1 * T2.inverse();

  const Precision dist =
      acos(T1.translation().normalized().dot(T2.translation().normalized())) * 180 / std::numbers::pi_v<Precision>;
  const Precision angle = diff.so3().log().norm() * 180 / std::numbers::pi_v<Precision>;
  LOG(INFO) << dist << " " << angle;
  return {dist, angle};
}

std::vector<std::pair<Precision, Precision>> evaluateError(size_t startFrame) {
  using Calibration = sensors::calibration::CameraCalibration;
  using Model = energy::model::PinholeCamera<Precision>;
  static constexpr energy::model::ModelType ModelType = energy::model::ModelType::kPinholeCamera;

  auto calibration = Calibration(Eigen::Vector2<Precision>(1280, 720),
                                 Eigen::Vector4<Precision>(448.15_p, 448.15_p, 640, 360), ModelType);
  auto model = calibration.cameraModel<Model>();
  auto init_model = calibration.cameraModel<Model>();
  auto photometric_calibration =
      sensors::calibration::photometric_calibration::create(TEST_DATA_DIR "track30seconds/pcalib.txt");
  auto vignetting = sensors::calibration::vignetting::create(TEST_DATA_DIR "track30seconds/vignetting.png");

  auto provider = std::make_unique<sensors::providers::ImageVideoProvider>(
      TEST_DATA_DIR "track30seconds/images.mkv", TEST_DATA_DIR "track30seconds/times.csv", startFrame);

  // create depth estimation
  Eigen::Vector2<Precision> im_size = calibration.image_size();
  auto camera_mask = sensors::calibration::CameraMask(static_cast<int>(im_size.y()), static_cast<int>(im_size.x()));

  auto settings = std::make_unique<sensors::calibration::CameraSettings>(
      std::move(calibration), std::move(photometric_calibration), std::move(vignetting), std::move(camera_mask));
  sensors::Camera camera("camera_1", 0, *settings, std::move(provider),
                         std::make_unique<dsopp::features::SobelTrackingFeaturesExtractor>());
  auto keyframe_strategy =
      std::make_unique<feature_based_slam::initialization_strategy::WaitForMovementInitializerKeyframeStrategy>(5, 0.7);

  auto features_extractor = std::make_unique<feature_based_slam::features::DistinctFeaturesExtractorORB>(500);

  std::function initializer_fabric = [&](feature_based_slam::track::Track &track,
                                         const feature_based_slam::features::DistinctFeaturesExtractor &featurer)
      -> std::unique_ptr<feature_based_slam::tracker::Initializer> {
    return std::make_unique<feature_based_slam::tracker::MonocularInitializer<Model>>(
        track, camera.id(), featurer, std::move(keyframe_strategy), feature_based_slam::tracker::Options(),
        std::move(init_model));
  };

  feature_based_slam::tracker::MonocularTracker<Model> monocular_initializer(camera.id(), std::move(features_extractor),
                                                                             initializer_fabric, std::move(model));

  const size_t kNumberOfThreads = 6;
  while (!monocular_initializer.initialized()) {
    dsopp::sensors::SynchronizedFrame frame(camera.nextFrameId(), camera.nextFrameTime());
    camera.processNextDataFrame(frame);
    monocular_initializer.tick(frame, kNumberOfThreads);
  }
  auto gt = dsopp::test_tools::TumGt(TEST_DATA_DIR "track30seconds/gt.tum");

  const size_t framesN = monocular_initializer.evaluatedFramesSize();

  const Sophus::SE3<Precision> &gt_cam0_world = gt.getPose(monocular_initializer.timestamp(0)).inverse();

  std::vector<std::pair<Precision, Precision>> errors;

  for (size_t i = 1; i < framesN; ++i) {
    if (!monocular_initializer.isInitialized(i)) continue;

    const Sophus::SE3<Precision> &gt_world_cam2 = gt.getPose(monocular_initializer.timestamp(i));
    Sophus::SE3<Precision> gt_pose = gt_cam0_world * gt_world_cam2;
    gt_pose.translation().normalize();
    Sophus::SE3<Precision> initialized_pose = monocular_initializer.tWorldAgent(i);
    initialized_pose.translation().normalize();
    errors.push_back(checkPosesEquality(gt_pose, initialized_pose));
  }
  return errors;
}

void printStatistic(const std::vector<Precision> &errs) {
  size_t size = errs.size();
  std::cout << "min \t 0.25 \t median \t 0.75 \t 0.9 \t max" << std::endl;
  std::cout << errs[0] << '\t' << errs[static_cast<size_t>(0.25 * static_cast<double>(size))] << '\t'
            << errs[static_cast<size_t>(0.5 * static_cast<double>(size))] << '\t'
            << errs[static_cast<size_t>(0.75 * static_cast<double>(size))] << '\t'
            << errs[static_cast<size_t>(0.9 * static_cast<double>(size))] << '\t' << errs[size - 1] << std::endl;
  ;
}

int main() {
  const size_t kNumberOfStarts = 50;
  const size_t kNumberOfFrames = 600;
  size_t step = kNumberOfFrames / kNumberOfStarts;

  std::vector<Precision> rotation_errors;
  std::vector<Precision> translation_errors;

  for (size_t first_frame = 0; first_frame < kNumberOfFrames; first_frame += step) {
    auto errors = evaluateError(static_cast<size_t>(first_frame));
    for (auto error : errors) {
      translation_errors.push_back(error.first);
      rotation_errors.push_back(error.second);
    }
  }
  std::sort(rotation_errors.begin(), rotation_errors.end());
  std::sort(translation_errors.begin(), translation_errors.end());

  std::cout << "\t####   ROTATION ERRORS  ####\n";
  printStatistic(rotation_errors);
  std::cout << "\t####   TRANSLATION ERRORS  ####\n";
  printStatistic(translation_errors);

  return 0;
}
