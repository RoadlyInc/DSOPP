#include "feature_based_slam/tracker/fabric.hpp"

#include <memory>
#include <sstream>

#include <Eigen/src/Core/MathFunctions.h>
#include <glog/logging.h>

#include "common/settings.hpp"
#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/camera_model/pinhole/simple_radial.hpp"
#include "feature_based_slam/features/distinct_features_extractor.hpp"
#include "feature_based_slam/features/distinct_features_extractor_orb.hpp"
#include "feature_based_slam/initialization_strategy/frequency_initializer_keyframe_strategy.hpp"
#include "feature_based_slam/initialization_strategy/initializer_keyframe_strategy.hpp"
#include "feature_based_slam/initialization_strategy/wait_for_movement_keyframe_strategy.hpp"
#include "feature_based_slam/tracker/monocular_initializer.hpp"
#include "feature_based_slam/tracker/monocular_tracker.hpp"
#include "feature_based_slam/tracker/void_initializer.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"

#include "common/fabric_tools/parameters.hpp"
#include "common/file_tools/read_tum_poses.hpp"
#include "sensors/camera/camera.hpp"

namespace dsopp::feature_based_slam::tracker {
namespace {

/**
 * Helper function to produce message with missing field
 */
std::string missingFieldMessage(const std::string &parameter, const std::string parameter_pack = "") {
  return "Missing field \"" + parameter + "\" in the monocular initializer " + parameter_pack +
         " parameters. Monocular "
         "Initializer was not created.";
}

std::unique_ptr<initialization_strategy::InitializerKeyframeStrategy> createInitializationKeyframeStrategy(
    const std::map<std::string, std::any> &parameters) {
  using common::fabric_tools::readParameter;
  // keyframe strategy
  if (parameters.count("keyframe_strategy") == 0) {
    LOG(WARNING) << missingFieldMessage("keyframe_strategy");
    return nullptr;
  }
  const auto &keyframe_strategy_parameters =
      std::any_cast<std::map<std::string, std::any>>(parameters.at("keyframe_strategy"));

  std::string keyframe_strategy_type;
  if (!readParameter(keyframe_strategy_type, keyframe_strategy_parameters, "strategy", false)) {
    LOG(WARNING) << missingFieldMessage("strategy", "keyframe");
    return nullptr;
  }

  if (keyframe_strategy_type == "frequency") {
    Precision frequency;
    if (!readParameter(frequency, keyframe_strategy_parameters, "frequency", false)) {
      LOG(WARNING) << missingFieldMessage("frequency", "keyframe");
      return nullptr;
    }

    return std::make_unique<initialization_strategy::FrequencyInitializerKeyframeStrategy>(frequency);
  } else if (keyframe_strategy_type == "wait_for_movement") {
    size_t sliding_window_length = 5;
    Precision rotation_inlier_ratio = 0.7_p;

    readParameter(sliding_window_length, keyframe_strategy_parameters, "sliding_window_length");
    readParameter(rotation_inlier_ratio, keyframe_strategy_parameters, "rotation_inlier_ratio");

    return std::make_unique<initialization_strategy::WaitForMovementInitializerKeyframeStrategy>(sliding_window_length,
                                                                                                 rotation_inlier_ratio);
  }
  LOG(WARNING) << "Unknow field \"strategy\" value : " << keyframe_strategy_type
               << " in the monocular initializer keyframe parameters. Monocular "
                  "Initializer was not created.";
  return nullptr;
}

std::unique_ptr<features::DistinctFeaturesExtractor> createFeatureExtractor(
    const std::map<std::string, std::any> &parameters) {
  using common::fabric_tools::readParameter;

  // features extractor
  if (!parameters.contains("features_extractor")) {
    LOG(WARNING) << missingFieldMessage("features_extractor");
    return nullptr;
  }
  const auto &features_extractor_parameters =
      std::any_cast<std::map<std::string, std::any>>(parameters.at("features_extractor"));

  std::string features_extractor_type;
  if (!readParameter(features_extractor_type, features_extractor_parameters, "type", false)) {
    LOG(WARNING) << missingFieldMessage("type", "features_extractor");
    return nullptr;
  }

  if (features_extractor_type == "ORB") {
    size_t number_of_features = 500;

    readParameter(number_of_features, features_extractor_parameters, "number_of_features");

    return std::make_unique<features::DistinctFeaturesExtractorORB>(number_of_features);
  }
  LOG(WARNING) << "Unknow field \"type\" value : " << features_extractor_type
               << " in the monocular initializer features extractor parameters. Monocular "
                  "Initializer was not created.";
  return nullptr;
}

template <energy::model::Model Model>
std::unique_ptr<Tracker> createMonocularTracker(const std::map<std::string, std::any> &parameters,
                                                const dsopp::sensors::Camera &camera_sensor) {
  using common::fabric_tools::readParameter;
  /*
   * Reading initializer class parameters
   */
  Options options;
  readParameter(options.reprojection_threshold, parameters, "reprojection_threshold");
  readParameter(options.rotation_ransac_iterations, parameters, "rotation_ransac_iterations");
  readParameter(options.min_ransac_matches, parameters, "min_ransac_matches");
  readParameter(options.se3_inlier_ratio, parameters, "se3_inlier_ratio");
  readParameter(options.essential_matrix_ransac_threshold, parameters, "essential_matrix_ransac_threshold");
  readParameter(options.triangulation_cos_threshold, parameters, "triangulation_cos_threshold");
  readParameter(options.pnp_inlier_ratio, parameters, "pnp_inlier_ratio");
  readParameter(options.pnp_ransac_threshold, parameters, "pnp_ransac_threshold");

  Eigen::Vector2<Precision> image_size = camera_sensor.calibration().image_size();
  const Precision kWidthDefault = 1280;
  const Precision kHeightDefault = 720;
  Precision width_resize = image_size[0] / kWidthDefault;
  Precision height_resize = image_size[1] / kHeightDefault;
  Precision threshold_scale = std::sqrt((width_resize * width_resize + height_resize * height_resize) / 2);

  options.essential_matrix_ransac_threshold *= threshold_scale;
  options.pnp_ransac_threshold *= threshold_scale;
  options.reprojection_threshold *= threshold_scale;

  auto keyframe_strategy = createInitializationKeyframeStrategy(parameters);
  auto feature_extractor = createFeatureExtractor(parameters);
  auto camera_model = camera_sensor.calibration().cameraModel<Model>();
  auto initializer_camera_model = camera_sensor.calibration().cameraModel<Model>();

  if (!feature_extractor || !camera_model || !initializer_camera_model) {
    return nullptr;
  }

  std::string initializer_type;
  if (!common::fabric_tools::readParameter(initializer_type, parameters, "initializer_type", false)) {
    LOG(ERROR) << "Missing field \"initializer_type\", Initializer wasn't created";
    return nullptr;
  }

  std::function<std::unique_ptr<Initializer>(track::Track &, const features::DistinctFeaturesExtractor &)>
      initializer_abstract_fabric;
  if (initializer_type == "calibrated") {
    if (!keyframe_strategy) {
      return nullptr;
    }
    initializer_abstract_fabric = [&](track::Track &track, const features::DistinctFeaturesExtractor &featurer) {
      return std::make_unique<MonocularInitializer<Model>>(track, camera_sensor.id(), featurer,
                                                           std::move(keyframe_strategy), options,
                                                           std::move(initializer_camera_model));
    };
  } else if (initializer_type == "autocalibration") {
    if (!keyframe_strategy) {
      return nullptr;
    }
    initializer_abstract_fabric = [&](track::Track &track, const features::DistinctFeaturesExtractor &featurer) {
      LOG(WARNING) << "contact Roadly INC for this functionality ";
      (void)track;
      (void)featurer;
      return nullptr;
    };
  } else if (initializer_type == "void") {
    size_t max_frames = 4;
    readParameter(max_frames, parameters, "max_frames", true);
    initializer_abstract_fabric = [&, max_frames = max_frames](track::Track &track,
                                                               const features::DistinctFeaturesExtractor &featurer) {
      return std::make_unique<VoidInitializer>(max_frames, track, camera_sensor.id(), featurer, options);
    };
  } else {
    LOG(INFO) << "Unsupported Initializer type " << initializer_type;
    return nullptr;
  }

  return std::make_unique<MonocularTracker<Model>>(camera_sensor.id(), std::move(feature_extractor),
                                                   initializer_abstract_fabric, std::move(camera_model), options);
}
}  // namespace

std::unique_ptr<Tracker> create(const std::map<std::string, std::any> &parameters, const sensors::Sensors &sensors) {
  std::string type;
  if (!common::fabric_tools::readParameter(type, parameters, "type", true)) {
    LOG(WARNING) << "Missing field \"type\" in the initializer parameters. Initializer was not created.";
    return nullptr;
  }

  if (type == "monocular") {
    if (!parameters.contains("sensor_id")) {
      LOG(WARNING) << "Missing field \"sensor_id\" in the initializer parameters. Initializer was not created.";
      return nullptr;
    }
    const auto &sensor_id = std::any_cast<std::string>(parameters.at("sensor_id"));
    auto *camera_sensor = sensors.getCamera(sensor_id);

    if (!camera_sensor) {
      LOG(WARNING) << "Sensor with id \"" << sensor_id
                   << "\" does not exists or not a camera. Tracker was not created.";
      return nullptr;
    }

    if (camera_sensor->calibration().type() == energy::model::ModelType::kPinholeCamera) {
      return createMonocularTracker<energy::model::PinholeCamera<Precision>>(parameters, *camera_sensor);
    } else if (camera_sensor->calibration().type() == energy::model::ModelType::kSimpleRadialCamera) {
      return createMonocularTracker<energy::model::SimpleRadialCamera<Precision>>(parameters, *camera_sensor);
    } else {
      LOG(ERROR) << "Initializer can only working with PinholeCamera model or SimpleRadialCamera model";
      return nullptr;
    }

  } else {
    LOG(WARNING) << "Undefined initializer type";
    return nullptr;
  }
}

}  // namespace dsopp::feature_based_slam::tracker
