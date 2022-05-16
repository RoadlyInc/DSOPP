#ifndef DSOPP_SRC_FEATURES_CAMERA_TRACKING_FEATURES_EXTRACTOR_HPP_
#define DSOPP_SRC_FEATURES_CAMERA_TRACKING_FEATURES_EXTRACTOR_HPP_

#include <memory>

#include <opencv2/opencv.hpp>

#include "common/settings.hpp"

namespace dsopp {
namespace sensors {
namespace calibration {
class CameraMask;
}  // namespace calibration
}  // namespace sensors

namespace features {
class TrackingFeaturesFrame;
/**
 * \brief TrackingFeatureExtractor extract feature points for tracking from frame data.
 *
 * Object of this class extract feature points from frame data for future
 * calculating transformation between images.
 */
class TrackingFeaturesExtractor {
 public:
  /**
   * Creates a tracking feature extractor.
   * @param point_density_for_detector desired number of the feature points
   */
  explicit TrackingFeaturesExtractor(const Precision point_density_for_detector = 1500)
      : point_density_for_detector_(point_density_for_detector) {}

  /**
   * Method to extract tracking features from image.
   *
   * @param image image
   * @param mask input image mask
   * @return tracking features frame which contain tracking features with coordinates
   */
  virtual std::unique_ptr<TrackingFeaturesFrame> extract(cv::Mat image,
                                                         const sensors::calibration::CameraMask &mask) = 0;

  virtual ~TrackingFeaturesExtractor() = default;

 protected:
  /** current potential control number of point that would be extracted. It's a dynamic parameter and it's update after
   * each extraction */
  int current_potential_ = 15;
  /** desired number of the feature points */
  Precision point_density_for_detector_;
  /** features are not extracted outside the border */
  const int kBorderSize = 4;
  /** mask erosion radius */
  const int kGradientBorder = 3;
};

}  // namespace features
}  // namespace dsopp

#endif  // DSOPP_SRC_FEATURES_CAMERA_TRACKING_FEATURES_EXTRACTOR_HPP_
