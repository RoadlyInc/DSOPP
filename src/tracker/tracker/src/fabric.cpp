
#include "tracker/fabric.hpp"

#include <fstream>
#include <string>

#include <glog/logging.h>

#include "common/fabric_tools/parameters.hpp"
#include "common/file_tools/read_tum_poses.hpp"
#include "common/settings.hpp"
#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/camera_model/pinhole/simple_radial.hpp"

#include "energy/problems/photometric_bundle_adjustment/ceres_photometric_bundle_adjustment.hpp"
#include "energy/problems/photometric_bundle_adjustment/eigen_photometric_bundle_adjustment.hpp"
#include "energy/problems/photometric_bundle_adjustment/photometric_bundle_adjustment.hpp"
#include "energy/problems/photometric_bundle_adjustment/void_photometric_bundle_adjustment.hpp"
#include "energy/problems/pose_alignment/eigen_pose_alignment.hpp"
#include "energy/problems/pose_alignment/pose_alignment.hpp"
#include "energy/problems/pose_alignment/precalculated_pose_alignment.hpp"
#include "features/camera/pixel_data_frame.hpp"
#include "marginalization/fabric.hpp"
#include "sensors/camera/camera.hpp"

#include "sensors/camera_calibration/camera_calibration.hpp"
#include "sensors/sensors.hpp"
#include "tracker/depth_estimators/depth_estimation.hpp"
#include "tracker/keyframe_strategy/fabric.hpp"
#include "tracker/keyframe_strategy/tracker_keyframe_strategy.hpp"
#include "tracker/landmarks_activator/landmarks_activator.hpp"
#include "tracker/monocular/monocular_tracker.hpp"
#include "tracker/tracker.hpp"

namespace dsopp {
namespace tracker {
namespace {
template <energy::motion::Motion Motion, energy::model::Model Model, int PatternSize, template <int> typename Grid2D,
          int C>
std::unique_ptr<
    energy::problem::PhotometricBundleAdjustment<Precision, Motion, Model, Pattern::kSize, Grid2D, true, true, true, C>>
createPhotometricBundleAdjustment(const std::map<std::string, std::any> &parameters, const Precision huber_loss_sigma) {
  const Precision kTrustRegionRadius = 1e5;
  const Precision kFunctionTolerance = 1e-8_p;
  const Precision kParameterTolerance = 1e-8_p;

  size_t max_iterations = 7;
  const Eigen::Vector2<Precision> kAffineBrightnessRegularizer = Eigen::Vector2<Precision>(0 * C, 0 * C);
  const Precision kFixedStateRegularizer = static_cast<Precision>(1e16 * C);
  const Precision huber_loss = static_cast<Precision>(huber_loss_sigma * std::sqrt(C));
  if (parameters.count("photometric_bundle_adjustment") == 0) {
    LOG(WARNING) << "Missing \"photometric_bundle_adjustment\" in the tracker parameters. Creating default one with "
                    "EigenPhotometricBundleAdjustment";
    return std::make_unique<
        energy::problem::EigenPhotometricBundleAdjustment<Motion, Model, PatternSize, Grid2D, true, true, true, C>>(
        energy::problem::TrustRegionPhotometricBundleAdjustmentOptions<Precision>(
            max_iterations, kTrustRegionRadius, kFunctionTolerance, kParameterTolerance, kAffineBrightnessRegularizer,
            kFixedStateRegularizer, huber_loss),
        true, true);
  }

  const auto &pba_parameters =
      std::any_cast<std::map<std::string, std::any>>(parameters.at("photometric_bundle_adjustment"));

  std::string solver_type;
  if (!common::fabric_tools::readParameter(solver_type, pba_parameters, "solver", false)) {
    LOG(ERROR) << "Missing field \"solver\". Photometric bundle adjustment was not created";
    return 0;
  }

  if (solver_type == "eigen") {
    common::fabric_tools::readParameter(max_iterations, pba_parameters, "max_iterations");
    return std::make_unique<
        energy::problem::EigenPhotometricBundleAdjustment<Motion, Model, PatternSize, Grid2D, true, true, true, C>>(
        energy::problem::TrustRegionPhotometricBundleAdjustmentOptions<Precision>(
            max_iterations, kTrustRegionRadius, kFunctionTolerance, kParameterTolerance, kAffineBrightnessRegularizer,
            kFixedStateRegularizer, huber_loss),
        true, true);

  } else if (solver_type == "ceres") {
    if constexpr (std::is_same_v<Precision, double>) {
      common::fabric_tools::readParameter(max_iterations, pba_parameters, "max_iterations");
      return std::make_unique<energy::problem::CeresPhotometricBundleAdjustment<Motion, Model, PatternSize, Grid2D,
                                                                                true, true, true, false, C>>(
          energy::problem::TrustRegionPhotometricBundleAdjustmentOptions<double>(
              max_iterations, kTrustRegionRadius, kFunctionTolerance, kParameterTolerance, kAffineBrightnessRegularizer,
              kFixedStateRegularizer, huber_loss_sigma * std::sqrt(C)),
          true, true, 0);
    } else {
      LOG(ERROR) << "Double as dsopp::Precision is only supported to create Ceres photometric bundle adjustment solver";
      return nullptr;
    }
  } else if (solver_type == "void") {
    return std::make_unique<energy::problem::VoidPhotometricBundleAdjustment<Motion, Model, C>>();
  }
  LOG(ERROR) << "Unsupported solver type " << solver_type;

  return nullptr;
}

template <energy::motion::Motion Motion, energy::model::Model Model, int PatternSize, template <int> typename Grid2D,
          int C>
std::unique_ptr<energy::problem::PoseAlignment<Motion, Model, PatternSize, Grid2D, C>> createPoseAlignment(
    const std::map<std::string, std::any> &parameters, const Precision huber_loss_sigma) {
  const Precision kInitialTrustRegionRadius = 1e2;
  const Precision kFunctionTolerance = 1e-5_p;
  const Precision kParameterTolerance = 1e-5_p;

  size_t max_iterations = 50;
  const Eigen::Vector2<Precision> kAffineBrightnessRegularizer = Eigen::Vector2<Precision>(0 * C, 0 * C);
  const Precision kFixedStateRegularizer = static_cast<Precision>(1e16 * C);
  const Precision huber_loss = static_cast<Precision>(huber_loss_sigma * std::sqrt(C));

  if (parameters.count("pose_alignment") == 0) {
    LOG(WARNING) << "Missing \"pose_alignment\" in the tracker parameters. Creating default one with "
                    "EigenPoseAlignment";
    return std::make_unique<energy::problem::EigenPoseAlignment<Motion, Model, PatternSize, Grid2D, C>>(
        energy::problem::TrustRegionPhotometricBundleAdjustmentOptions<Precision>(
            max_iterations, kInitialTrustRegionRadius, kFunctionTolerance, kParameterTolerance,
            kAffineBrightnessRegularizer, kFixedStateRegularizer, huber_loss));
  }

  const auto &pose_alignment_parameters =
      std::any_cast<std::map<std::string, std::any>>(parameters.at("pose_alignment"));

  std::string solver_type;
  if (!common::fabric_tools::readParameter(solver_type, pose_alignment_parameters, "solver", false)) {
    LOG(ERROR) << "Missing field \"solver\". Pose alignment was not created";
    return 0;
  }

  if (solver_type == "eigen") {
    common::fabric_tools::readParameter(max_iterations, pose_alignment_parameters, "max_iterations");
    return std::make_unique<energy::problem::EigenPoseAlignment<Motion, Model, PatternSize, Grid2D, C>>(
        energy::problem::TrustRegionPhotometricBundleAdjustmentOptions<Precision>(
            max_iterations, kInitialTrustRegionRadius, kFunctionTolerance, kParameterTolerance,
            kAffineBrightnessRegularizer, kFixedStateRegularizer, huber_loss));

  } else if (solver_type == "precalculated") {
    std::string poses_file_path;
    if (!common::fabric_tools::readParameter(poses_file_path, pose_alignment_parameters, "poses_file", false)) {
      LOG(ERROR) << "In order to use precalculated poses file with them should be provided in \"poses_file\"  ";
      return nullptr;
    }
    std::ifstream file(poses_file_path);
    if (!file.is_open()) {
      LOG(ERROR) << poses_file_path << " is invalid file";
      return nullptr;
    }
    auto timestamp_poses = common::file_tools::readTumPoses<Motion>(file);
    return std::make_unique<energy::problem::PrecalculatedPoseAlignment<Motion, Model, PatternSize, C>>(
        std::move(timestamp_poses));
  }
  LOG(ERROR) << "Unsupported solver type " << solver_type;

  return nullptr;
}

template <energy::motion::Motion Motion, energy::model::Model Model, typename DepthEstimator,
          template <int> typename Grid2D, bool FRAME_EMBEDDER>
std::unique_ptr<Tracker<Motion>> createMonocularTracker(
    const std::map<std::string, std::any> &parameters, const sensors::Camera &camera, bool use_imu_prior,
    bool save_images_to_track,
    std::unique_ptr<keyframe_strategy::TrackerKeyframeStrategy<Motion>> &&tracker_keyframe_strategy,
    std::unique_ptr<marginalization::FrameMarginalizationStrategy<Motion>> &&frame_marginalization_strategy) {
  using TrackerType = MonocularTracker<Motion, Model, DepthEstimator, Grid2D, FRAME_EMBEDDER>;

  auto photomeric_bundle_adjustment =
      createPhotometricBundleAdjustment<Motion, Model, Pattern::kSize, Grid2D, TrackerType::C>(
          parameters, TrackerType::kHuberLossSigma);
  if (!photomeric_bundle_adjustment) {
    LOG(WARNING) << "Photometric bundle adjustment solver was not created.";
    return nullptr;
  }

  auto pose_alignment =
      createPoseAlignment<Motion, Model, TrackerType::kFrontendPatternSize, Grid2D,
                          TrackerType::kFrontendChannelsNumber>(parameters, TrackerType::kHuberLossSigma);
  if (!pose_alignment) {
    LOG(WARNING) << "Pose alignment solver was not created.";
    return nullptr;
  }

  size_t number_of_desired_points = 2000;
  common::fabric_tools::readParameter(number_of_desired_points, parameters, "number_of_desired_points");

  auto landmarks_activator = std::make_unique<LandmarksActivator<Motion, Model, Grid2D, 1, true>>(
      camera.calibration(), TrackerType::kHuberLossSigma, number_of_desired_points);

  return std::make_unique<TrackerType>(camera.id(), camera.calibration(), use_imu_prior, save_images_to_track,
                                       std::move(tracker_keyframe_strategy), std::move(frame_marginalization_strategy),
                                       std::move(photomeric_bundle_adjustment), std::move(pose_alignment),
                                       std::move(landmarks_activator));
}

template <energy::motion::Motion Motion, typename DepthEstimator, template <int> typename Grid2D, bool FRAME_EMBEDDER>
std::unique_ptr<Tracker<Motion>> createWithCalibration(const std::map<std::string, std::any> &parameters,
                                                       const sensors::Camera &camera) {
  if (parameters.count("keyframe_strategy") == 0) {
    LOG(WARNING) << "Missing field \"keyframe_strategy\" in the tracker parameters. Tracker was not created.";
    return nullptr;
  }
  const auto &tracker_keyframe_strategy_parameters =
      std::any_cast<std::map<std::string, std::any>>(parameters.at("keyframe_strategy"));
  auto tracker_keyframe_strategy =
      keyframe_strategy::createTrackerKeyframeStrategy<Motion>(tracker_keyframe_strategy_parameters);
  if (tracker_keyframe_strategy == nullptr) {
    LOG(WARNING) << "Impossible to create tracker keyframe strategy. Tracker was not created.";
    return nullptr;
  }

  if (parameters.count("marginalization_strategy") == 0) {
    LOG(WARNING) << "Missing field \"marginalization_strategy\" in the tracker parameters. Tracker was not created.";
    return nullptr;
  }
  const auto &marginalization_parameters =
      std::any_cast<std::map<std::string, std::any>>(parameters.at("marginalization_strategy"));
  auto frame_marginalization_strategy = marginalization::create<Motion>(marginalization_parameters);
  if (frame_marginalization_strategy == nullptr) {
    LOG(WARNING) << "Impossible to create tracker keyframe marginalization strategy. Tracker was not created.";
    return nullptr;
  }

  bool use_imu_prior = common::fabric_tools::readSwitcherParameter(parameters, "use_imu_prior");
  bool save_images_to_track = common::fabric_tools::readSwitcherParameter(parameters, "save_images");

  if (camera.calibration().type() == energy::model::ModelType::kPinholeCamera) {
    using Model = energy::model::PinholeCamera<Precision>;

    return createMonocularTracker<Motion, Model, DepthEstimator, Grid2D, FRAME_EMBEDDER>(
        parameters, camera, use_imu_prior, save_images_to_track, std::move(tracker_keyframe_strategy),
        std::move(frame_marginalization_strategy));

  } else if (camera.type() == energy::model::ModelType::kSimpleRadialCamera) {
    using Model = energy::model::SimpleRadialCamera<Precision>;

    return createMonocularTracker<Motion, Model, DepthEstimator, Grid2D, FRAME_EMBEDDER>(
        parameters, camera, use_imu_prior, save_images_to_track, std::move(tracker_keyframe_strategy),
        std::move(frame_marginalization_strategy));
  } else {
    LOG(ERROR) << "Monocular tracker can only working with pinhole model or simple radial model";
    return nullptr;
  }
}

template std::unique_ptr<Tracker<energy::motion::SE3<Precision>>>
createWithCalibration<energy::motion::SE3<Precision>, tracker::DepthEstimation, features::PixelMap, false>(
    const std::map<std::string, std::any> &, const sensors::Camera &);

template std::unique_ptr<Tracker<energy::motion::SE3<Precision>>>
createWithCalibration<energy::motion::SE3<Precision>, tracker::DepthEstimation, features::PixelMap, true>(
    const std::map<std::string, std::any> &, const sensors::Camera &);

}  // namespace
template <energy::motion::Motion Motion, typename DepthEstimator, template <int> typename Grid2D>
requires tracker::DepthEstimator<DepthEstimator, typename Motion::Product> std::unique_ptr<Tracker<Motion>> create(
    const std::map<std::string, std::any> &parameters, const sensors::Sensors &sensors) {
  if (parameters.count("type") == 0) {
    LOG(WARNING) << "Missing field \"type\" in the tracker parameters. Tracker was not created.";
    return nullptr;
  }

  const auto &type = std::any_cast<std::string>(parameters.at("type"));

  if (type == "monocular") {
    if (parameters.count("sensor_id") == 0) {
      LOG(WARNING) << "Missing field \"sensor_id\" in the tracker parameters. Tracker was not created.";
      return nullptr;
    }
    const auto &sensor_id = std::any_cast<std::string>(parameters.at("sensor_id"));
    auto *camera_sensor = sensors.getCamera(sensor_id);
    if (!camera_sensor) {
      LOG(WARNING) << "Sensor with id \"" << sensor_id
                   << "\" does not exists or not a camera. Tracker was not created.";
      return nullptr;
    }

    bool frame_embedder = false;
    if (parameters.count("frame_embedder") != 0) {
      frame_embedder = std::any_cast<std::string>(parameters.at("frame_embedder")) == "on";
    }

    if (frame_embedder) {
      return createWithCalibration<Motion, DepthEstimator, Grid2D, true>(parameters, *camera_sensor);
    } else {
      return createWithCalibration<Motion, DepthEstimator, Grid2D, false>(parameters, *camera_sensor);
    }

  } else {
    LOG(WARNING) << "Undefined tracker type";
    return nullptr;
  }
}

template std::unique_ptr<Tracker<energy::motion::SE3<Precision>>>
create<energy::motion::SE3<Precision>, tracker::DepthEstimation, features::PixelMap>(
    const std::map<std::string, std::any> &, const sensors::Sensors &);

}  // namespace tracker
}  // namespace dsopp
