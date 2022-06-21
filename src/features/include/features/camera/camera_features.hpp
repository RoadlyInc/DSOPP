#ifndef DSOPP_CAMERA_FEATURES_HPP
#define DSOPP_CAMERA_FEATURES_HPP

#include <memory>

#include <opencv2/opencv.hpp>

#include "common/settings.hpp"
#include "common/time/time.hpp"

namespace dsopp {
namespace semantics {
class SemanticLegend;
class SemanticFilter;
}  // namespace semantics
namespace sensors {

namespace calibration {
class CameraMask;
}  // namespace calibration
}  // namespace sensors
namespace features {
class TrackingFeaturesFrame;
class TrackingFeaturesExtractor;
template <int>
class PixelMap;
class PixelDataFrame;
class PixelDataFrameExtractor;
template <int>
class FrameEmbeddingExtractor;
/**
 * object to store camera frame and produce features on demand
 */
class CameraFeatures {
 public:
  /** Alias for pyramid of masks.*/
  using PyramidOfMasks = std::unique_ptr<std::vector<sensors::calibration::CameraMask>>;
  /**
   * Creating a camera features frame from frame data and an identifier.
   *
   * @param frame_id frame identifier
   * @param raw_image raw image loaded by data provider
   * @param time time when sensor captured data
   * @param pyramid_of_static_masks pyramid of static camera masks at the current time
   * @param tracking_features_extractor, pixel_data_frame_extractor,
   * @param semantics_data semantics data
   * @param semantic_filter filter legend (if exists, may be nullptr)
   */
  CameraFeatures(const size_t frame_id, cv::Mat&& raw_image, const time time,
                 const std::vector<sensors::calibration::CameraMask>& pyramid_of_static_masks,
                 features::TrackingFeaturesExtractor& tracking_features_extractor,
                 const features::PixelDataFrameExtractor& pixel_data_frame_extractor,
                 std::unique_ptr<cv::Mat>&& semantics_data = nullptr,
                 const semantics::SemanticFilter* semantic_filter = nullptr);
  /**
   * @return timestamp of the data
   */
  time timestamp() const;
  /**
   * @return frame id
   */
  size_t id() const;
  /**
   * @return tracking features
   */
  const features::TrackingFeaturesFrame& tracking();
  /**
   * @return pyramids
   */
  const features::PixelDataFrame& pixelData();
  /**
   * @return raw image (from sensor)
   */
  const cv::Mat& image() const;
  /**
   * @return frame data
   */
  const cv::Mat& frameData() const;
  /**
   * moves semantics data
   * @return moved semantic data
   */
  std::unique_ptr<cv::Mat> moveSemanticsData();
  /*
   *  TODO: delete move methods methods after RLY-45
   */
  /**
   * moves tracking features from the object
   * @return tracking features
   */
  std::unique_ptr<features::TrackingFeaturesFrame> moveTracking();
  /**
   * moves pyramids from the object
   * @return pyramids
   */
  std::unique_ptr<features::PixelDataFrame> movePixelData();
  /**
   * moves pyramid of masks from the object
   * @return pyramid of masks
   */
  PyramidOfMasks movePyramidOfMasks();
  /**
   * @return pyramid of masks
   */
  const std::vector<sensors::calibration::CameraMask>& pyramidOfMasks();

  ~CameraFeatures();

 private:
  /** frame identifier. */
  size_t frame_id_;
  /** The main container which contains raw image. */
  cv::Mat raw_image_;
  /** frame data. */
  cv::Mat frame_data_;
  /** time of the frame. */
  time time_;
  /** camera mask at the current time. */
  const std::vector<sensors::calibration::CameraMask>& pyramid_of_static_masks_;
  /** cached tracking features */
  std::unique_ptr<TrackingFeaturesFrame> tracking_features_;
  /** cached pyramids */
  std::unique_ptr<PixelDataFrame> pixel_data_frame_;
  /** extractor for tracking features */
  features::TrackingFeaturesExtractor& tracking_features_extractor_;
  /** extractor for pyramids */
  const features::PixelDataFrameExtractor& pixel_data_frame_extractor_;
  /** Semantics data. */
  std::unique_ptr<cv::Mat> semantics_data_;
  /** filter legend (if exists, may be nullptr) */
  const semantics::SemanticFilter* semantic_filter_;
  /** Camera mask */
  PyramidOfMasks pyramid_of_masks_;
};
}  // namespace features
}  // namespace dsopp

#endif  // DSOPP_CAMERA_FEATURES_HPP
