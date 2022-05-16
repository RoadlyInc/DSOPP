
#ifndef DSOPP_CAMERA_HPP
#define DSOPP_CAMERA_HPP

#include <memory>

#include <opencv2/opencv.hpp>

#include "common/settings.hpp"
#include "sensor/sensor.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"
#include "sensors/camera_calibration/camera_settings.hpp"
#include "sensors/camera_transformers/camera_transformer.hpp"

namespace dsopp {
namespace features {
class TrackingFeaturesExtractor;
class PixelDataFrameExtractor;
template <int>
class FrameEmbeddingExtractor;
}  // namespace features

namespace semantics {
class SemanticFilter;
}

namespace sensors {
namespace calibration {
class CameraMask;
}  // namespace calibration

namespace providers {
class CameraDataFrame;
class CameraProvider;
}  // namespace providers

/**
 * \brief Sensor that works with a sequence of images.
 */
class Camera final : public Sensor {
 public:
  /**
   * creates Camera with a given configuration
   * @param name unique identifier for sensor
   * @param id unique identifier for sensor
   * @param camera_settings camera settings with the calibration, photometric_calibration, mask and vignetting
   * @param provider object that provides images to the camera
   * @param transformers camera transformers
   * @param semantics_provider object that provides semantics data to the camera
   * @param semantic_filter semantics filter
   * @param tracking_feature_extractor tracking feature extractor
   */
  Camera(const std::string &name, size_t id, const calibration::CameraSettings &camera_settings,
         std::unique_ptr<providers::CameraProvider> &&provider,
         std::unique_ptr<features::TrackingFeaturesExtractor> &&tracking_feature_extractor,
         std::vector<std::unique_ptr<camera_transformers::CameraTransformer>> &&transformers = {},
         std::unique_ptr<providers::CameraProvider> &&semantics_provider = nullptr,
         std::unique_ptr<semantics::SemanticFilter> &&semantic_filter = nullptr);

  bool processNextDataFrame(sensors::SynchronizedFrame &frame) override;

  /**
   * method to access index of next frame
   * @return index of next frame
   */
  int nextFrameId() override;

  /**
   * method to access time of next frame
   * @return time of next frame
   */
  time nextFrameTime() override;

  /**
   * type of a camera model
   * @return type of a camera model (pinhole, simple radial, etc.)
   */
  energy::model::ModelType type() const;

  /**
   * method to know if camera stream ended
   * @return true if camera stream ended and false otherwise
   */
  bool empty() override;

  /**
   * @return camera calibration
   */
  const calibration::CameraCalibration &calibration() const;

  /**
   * @return camera settings
   */
  const calibration::CameraSettings &settings() const;

  /**
   * @return camera mask
   */
  const std::vector<calibration::CameraMask> &pyramidOfMasks() const;

  ~Camera() override;

 private:
  /** index of next frame.*/
  int next_frame_id_;
  /** time of next frame.*/
  time next_frame_time_;
  /** next frame.*/
  std::unique_ptr<providers::CameraDataFrame> next_frame_;
  /** stores all camera calibration settings */
  const calibration::CameraSettings &camera_settings_;
  /** Object that provides images to the camera.*/
  std::unique_ptr<providers::CameraProvider> provider_;
  /** Tracking features extractor.*/
  std::unique_ptr<features::TrackingFeaturesExtractor> tracking_feature_extractor_;
  /** pyramids extractor*/
  std::unique_ptr<features::PixelDataFrameExtractor> pixel_data_frame_extractor_;
  /** Pyramid of static masks */
  std::vector<calibration::CameraMask> pyramid_of_static_masks_;
  /** Object that provides semantics data to the camera */
  std::unique_ptr<providers::CameraProvider> semantics_provider_;
  /** semantic filter */
  std::unique_ptr<semantics::SemanticFilter> semantic_filter_;
  /** camera transformers */
  std::vector<std::unique_ptr<camera_transformers::CameraTransformer>> transformers_;
};

}  // namespace sensors
}  // namespace dsopp

#endif  // DSOPP_CAMERA_HPP
