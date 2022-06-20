#include "sensors/camera/camera.hpp"

#include <memory>

#include <glog/logging.h>

#include "features/camera/camera_features.hpp"
#include "features/camera/frame_embedding_extractor.hpp"
#include "features/camera/photometrically_corrected_image.hpp"
#include "features/camera/pixel_data_frame.hpp"
#include "features/camera/pixel_data_frame_extractor.hpp"
#include "features/camera/pixel_map.hpp"
#include "features/camera/tracking_features_extractor.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"
#include "sensors/camera_calibration/mask/camera_mask.hpp"

#include "semantics/semantic_filter.hpp"
#include "sensor/synchronized_frame.hpp"
#include "sensor/utilities.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"
#include "sensors/camera_calibration/mask/camera_mask.hpp"
#include "sensors/camera_providers/camera_data_frame.hpp"
#include "sensors/camera_providers/camera_provider.hpp"

namespace dsopp {
namespace sensors {

Camera::Camera(const std::string &name, size_t id, const calibration::CameraSettings &camera_settings,
               std::unique_ptr<providers::CameraProvider> &&provider,
               std::unique_ptr<features::TrackingFeaturesExtractor> &&tracking_feature_extractor,
               std::vector<std::unique_ptr<camera_transformers::CameraTransformer>> &&transformers,
               std::unique_ptr<providers::CameraProvider> &&semantics_provider,
               std::unique_ptr<semantics::SemanticFilter> &&semantic_filter)
    : Sensor(name, id),
      camera_settings_(camera_settings),
      provider_(std::move(provider)),
      tracking_feature_extractor_(std::move(tracking_feature_extractor)),
      semantics_provider_(std::move(semantics_provider)),
      semantic_filter_(std::move(semantic_filter)),
      transformers_(std::move(transformers)) {
  const auto &mask = camera_settings_.cameraMask();
  for (size_t lvl = 0; lvl < camera_settings_.calibration().kNumberOfPyramidLevels; lvl++) {
    pyramid_of_static_masks_.emplace_back(mask.resize(1._p / static_cast<Precision>(1 << lvl)));
  }

  pixel_data_frame_extractor_ =
      std::make_unique<features::PixelDataFrameExtractor>(calibration::CameraCalibration::kNumberOfPyramidLevels);
}

bool Camera::processNextDataFrame(sensors::SynchronizedFrame &frame) {
  utilities::fillNextFrame(next_frame_, next_frame_id_, next_frame_time_, *provider_);
  auto next_frame = std::move(next_frame_);
  if (!next_frame) {
    return false;
  }

  const auto &undistorter = camera_settings_.calibration().undistorter();

  std::unique_ptr<cv::Mat> semantics_data = nullptr;
  if (semantics_provider_) {
    auto semantics_frame = semantics_provider_->nextFrame();
    if (semantics_frame) {
      semantics_data =
          std::make_unique<cv::Mat>(runMaskTransformers(transformers_, undistorter.undistort(semantics_frame->data())));
      CHECK(semantics_frame->timestamp() == next_frame->timestamp());
    }
  }

  cv::Mat image = next_frame->data();
  cv::Mat transformed_image = runImageTransformers(transformers_, image);
  cv::Mat raw_undistorted_image = undistorter.undistort(transformed_image);
  auto width = static_cast<int>(transformed_image.cols);
  auto height = static_cast<int>(transformed_image.rows);
  cv::Mat transformed_image_grayscale;
  cv::cvtColor(transformed_image, transformed_image_grayscale, cv::COLOR_BGR2GRAY);
  std::vector<Precision> photocorrected_image_vector = features::photometricallyCorrectedImage(
      transformed_image_grayscale, camera_settings_.photometricCalibration(), camera_settings_.vignetting());
  cv::Mat photocorrected_image(height, width, std::is_same_v<dsopp::Precision, double> ? CV_64FC1 : CV_32FC1,
                               photocorrected_image_vector.data());
  cv::Mat photocorrected_undistorted_image = undistorter.undistort(photocorrected_image);

  frame.addCameraFeatures(
      this->id(),
      std::make_unique<features::CameraFeatures>(
          next_frame->id(), std::move(raw_undistorted_image), std::move(photocorrected_undistorted_image),
          next_frame->timestamp(), pyramid_of_static_masks_, *tracking_feature_extractor_, *pixel_data_frame_extractor_,
          frame_embedding_extractor_.get(), std::move(semantics_data), semantic_filter_.get()));
  return true;
}

int Camera::nextFrameId() {
  utilities::fillNextFrame(next_frame_, next_frame_id_, next_frame_time_, *provider_);
  return next_frame_id_;
}

time Camera::nextFrameTime() {
  utilities::fillNextFrame(next_frame_, next_frame_id_, next_frame_time_, *provider_);
  return next_frame_time_;
}

bool Camera::empty() {
  utilities::fillNextFrame(next_frame_, next_frame_id_, next_frame_time_, *provider_);
  return !next_frame_;
}

energy::model::ModelType Camera::type() const { return camera_settings_.calibration().type(); }

const calibration::CameraCalibration &Camera::calibration() const { return camera_settings_.calibration(); }

const calibration::CameraSettings &Camera::settings() const { return camera_settings_; }

const std::vector<calibration::CameraMask> &Camera::pyramidOfMasks() const { return pyramid_of_static_masks_; }

Camera::~Camera() = default;
}  // namespace sensors
}  // namespace dsopp
