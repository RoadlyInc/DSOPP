#include "sensors/sensors_builder/camera_fabric.hpp"

#include <functional>
#include <memory>
#include <set>

#include <glog/logging.h>
#include <Eigen/Dense>
#include <filesystem>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgcodecs.hpp>

#include "common/fabric_tools/parameters.hpp"
#include "common/file_tools/parsing.hpp"
#include "common/settings.hpp"
#include "features/camera/eigen_tracking_features_extractor.hpp"
#include "features/camera/frame_embedding_extractor.hpp"
#include "features/camera/sobel_tracking_features_extractor.hpp"
#include "semantics/semantic_filter.hpp"
#include "sensors/camera/camera.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"
#include "sensors/camera_calibration/fabric.hpp"
#include "sensors/camera_calibration/mask/camera_mask.hpp"
#include "sensors/camera_providers/camera_provider.hpp"
#include "sensors/camera_providers/fabric.hpp"
#include "sensors/camera_transformers/fabric.hpp"

namespace dsopp {
namespace sensors {

std::unique_ptr<Camera> createCamera(
    const std::map<std::string, std::any> &parameters, const std::string &sensor_name, size_t sensor_id,
    std::vector<std::unique_ptr<camera_transformers::CameraTransformer>> &&transformers,
    const calibration::CameraSettings &camera_settings) {
  auto provider = providers::createCameraProvider(parameters);
  if (provider == nullptr) {
    LOG(ERROR) << "impossible to create provider. Camera was not created";
    return nullptr;
  }

  if (parameters.contains("frame_embedder")) {
    const auto &frame_embedder = std::any_cast<std::map<std::string, std::any>>(parameters.at("frame_embedder"));
    std::string frame_embedder_type = "identity";
    common::fabric_tools::readParameter(frame_embedder_type, frame_embedder, "type");
    if (frame_embedder_type == "gn_net") {
      LOG(WARNING) << "contact Roadly INC for this functionality";
    } else if (frame_embedder_type == "identity") {
      LOG(WARNING) << "contact Roadly INC for this functionality";
    } else {
      LOG(FATAL) << "Undefined frame embedder type";
    }
  }

  std::unique_ptr<providers::CameraProvider> semantics_provider = nullptr;
  bool filter_by_semantic = false;
  std::unique_ptr<semantics::SemanticFilter> semantic_filter = nullptr;

  if (parameters.contains("segmentation")) {
    const auto &segmentation = std::any_cast<std::map<std::string, std::any>>(parameters.at("segmentation"));

    if (segmentation.contains("provider")) {
      const auto &semantics_parameters = std::any_cast<std::map<std::string, std::any>>(parameters.at("segmentation"));
      semantics_provider = providers::createCameraProvider(semantics_parameters, true);

      if (semantics_provider == nullptr) {
        LOG(ERROR) << "impossible to create semantics provider.";
      } else if (segmentation.contains("semantic_legend")) {
        bool filter_indicated_objects = true;

        filter_by_semantic = common::fabric_tools::readSwitcherParameter(segmentation, "filter_by_semantic");

        std::vector<std::string> filter_names;
        if (segmentation.contains("objects_to_filter")) {
          std::string objects_to_filter = std::any_cast<std::string>(segmentation.at("objects_to_filter"));
          filter_names = common::file_tools::splitLine(objects_to_filter, ' ');
        }

        std::string filter_type;
        if (!common::fabric_tools::readParameter(filter_type, segmentation, "filter_type", false)) {
          LOG(ERROR)
              << "\"filter_type\" is not provided. Filtration wouldn't be done. Allowed values are : include/exclude";
          filter_names.clear();
          filter_by_semantic = false;
        } else {
          if (filter_type == "exclude") {
            filter_indicated_objects = true;
          } else if (filter_type == "include") {
            filter_indicated_objects = false;
          } else {
            LOG(ERROR) << "Unknown filter type: " << filter_type << " include/exclude are only supported";
            filter_names.clear();
            filter_by_semantic = false;
          }
        }
        if (camera_settings.semanticLegend()) {
          semantic_filter = std::make_unique<semantics::SemanticFilter>(*camera_settings.semanticLegend(), filter_names,
                                                                        filter_indicated_objects, filter_by_semantic);
        }
      }
    }
  }

  std::string tracking_feature_extractor_type = "sobel";
  Precision point_density_for_detector = 1500;
  Precision quantile_level = 0.8_p;
  Precision weight_of_mean = 1.0_p;
  if (parameters.contains("features_extractor")) {
    const auto &features_extractor =
        std::any_cast<std::map<std::string, std::any>>(parameters.at("features_extractor"));
    common::fabric_tools::readParameter(tracking_feature_extractor_type, features_extractor, "type");
    common::fabric_tools::readParameter(point_density_for_detector, features_extractor, "point_density_for_detector");
    if (tracking_feature_extractor_type == "sobel") {
      common::fabric_tools::readParameter(quantile_level, features_extractor, "quantile_level");
      common::fabric_tools::readParameter(weight_of_mean, features_extractor, "weight_of_mean");
    }
  } else {
    LOG(WARNING) << "Missing field \"features_extractor\"";
  }

  std::unique_ptr<features::TrackingFeaturesExtractor> tracking_feature_extractor;
  if (tracking_feature_extractor_type == "sobel") {
    tracking_feature_extractor = std::make_unique<features::SobelTrackingFeaturesExtractor>(
        point_density_for_detector, quantile_level, weight_of_mean);
  } else if (tracking_feature_extractor_type == "eigen") {
    tracking_feature_extractor = std::make_unique<features::EigenTrackingFeaturesExtractor>(point_density_for_detector);
  } else {
    LOG(FATAL) << "Undefined tracking feature extractor type";
  }
  return std::make_unique<Camera>(sensor_name, sensor_id, camera_settings, std::move(provider),
                                  std::move(tracking_feature_extractor), std::move(transformers),
                                  std::move(semantics_provider), std::move(semantic_filter));
}

template <bool TRANSFORM_TO_PINHOLE>
std::optional<calibration::CameraSettings> createCameraSettings(
    const std::map<std::string, std::any> &parameters,
    const std::vector<std::unique_ptr<camera_transformers::CameraTransformer>> &transformers) {
  if (!parameters.contains("model")) {
    LOG(WARNING) << "Missing field \"model\" in the sensor parameters";
    return std::nullopt;
  }
  const auto &model_parameters = std::any_cast<std::map<std::string, std::any>>(parameters.at("model"));

  auto calibration = calibration::camera_calibration::create(model_parameters, TRANSFORM_TO_PINHOLE);
  if (!calibration) {
    LOG(ERROR) << "impossible to create camera calibration. Camera was not created";
    return std::nullopt;
  }
  auto photometric_calibration = calibration::photometric_calibration::create(model_parameters);
  auto vignetting = calibration::vignetting::create(model_parameters);

  auto camera_mask = calibration::mask::create(parameters, static_cast<int>(calibration->undistorter().input_width()),
                                               static_cast<int>(calibration->undistorter().input_height()));
  if (!camera_mask) {
    return std::nullopt;
  }

  // transform
  for (auto &transformer : transformers) {
    transformer->transformCalibration(*calibration);
  }

  /** Undistort mask */
  const auto &undistorter = calibration->undistorter();

  camera_mask = calibration::CameraMask(runMaskTransformers(transformers, undistorter.undistort(camera_mask->data())));

  // read sematincs legend
  std::unique_ptr<semantics::SemanticLegend> semantic_legend = nullptr;

  if (parameters.contains("segmentation")) {
    const auto &segmentation = std::any_cast<std::map<std::string, std::any>>(parameters.at("segmentation"));
    if (segmentation.contains("semantic_legend")) {
      std::ifstream semantic_legend_file(std::any_cast<std::string>(segmentation.at("semantic_legend")));
      if (semantic_legend_file.is_open()) {
        semantic_legend = std::make_unique<semantics::SemanticLegend>(semantic_legend_file);
      } else {
        LOG(ERROR) << "Unable to open semantic_legen file";
      }
    } else {
      LOG(ERROR) << "There is no semantic legend or remove_dynamic_objects flag";
    }
  }

  // TODO:fix
  return std::optional<calibration::CameraSettings>(std::in_place, std::move(*calibration),
                                                    std::move(photometric_calibration), std::move(vignetting),
                                                    std::move(*camera_mask), std::move(semantic_legend));
}

std::vector<std::unique_ptr<camera_transformers::CameraTransformer>> createCameraTransformer(
    const std::map<std::string, std::any> &parameters) {
  std::vector<std::unique_ptr<camera_transformers::CameraTransformer>> transformers;
  if (parameters.contains("transformations")) {
    const auto &transformer_parameters =
        std::any_cast<std::map<std::string, std::any>>(parameters.at("transformations"));

    transformers = camera_transformers::create(transformer_parameters);
  }
  return transformers;
}

template std::optional<calibration::CameraSettings> createCameraSettings<true>(
    const std::map<std::string, std::any> &parameters,
    const std::vector<std::unique_ptr<camera_transformers::CameraTransformer>> &transformers);
template std::optional<calibration::CameraSettings> createCameraSettings<false>(
    const std::map<std::string, std::any> &parameters,
    const std::vector<std::unique_ptr<camera_transformers::CameraTransformer>> &transformers);
}  // namespace sensors
}  // namespace dsopp
