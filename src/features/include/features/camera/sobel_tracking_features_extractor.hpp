#ifndef DSOPP_SRC_FEATURES_CAMERA_SOBEL_TRACKING_FEATURES_EXTRACTOR_HPP
#define DSOPP_SRC_FEATURES_CAMERA_SOBEL_TRACKING_FEATURES_EXTRACTOR_HPP

#include "features/camera/tracking_features_extractor.hpp"

#include <memory>
#include <string>

#include "common/settings.hpp"

namespace dsopp {
namespace features {
/**
 * \brief TrackingFeatureExtractor extract feature points for tracking from frame data.
 *
 * Object of this class extract feature points from frame data for future
 * calculating transformation between images.
 */
class SobelTrackingFeaturesExtractor : public TrackingFeaturesExtractor {
 public:
  /**
   * Creates a tracking feature extractor.
   * @param point_density_for_detector desired number of the feature points
   * @param quantile_level quantile level for gradient norm threshold
   * @param weight_of_mean weight of mean value of gradient in block for gradient threshold
   */
  explicit SobelTrackingFeaturesExtractor(const Precision point_density_for_detector = 1500,
                                          const Precision quantile_level = 0.8_p,
                                          const Precision weight_of_mean = 1.0_p);

  std::unique_ptr<TrackingFeaturesFrame> extract(cv::Mat image, const sensors::calibration::CameraMask &mask) override;

 private:
  /** quantile level for gradient norm threshold */
  Precision quantile_level_;
  /** weight of mean value of gradient in block for gradient threshold */
  Precision weight_of_mean_;
  /** minimum of gradient norm */
  Precision grad_norm_threshold_;
  /** ``true`` after initialization of the all fields */
  bool initialized_ = false;
};

}  // namespace features
}  // namespace dsopp

#endif  // DSOPP_SRC_FEATURES_CAMERA_SOBEL_TRACKING_FEATURES_EXTRACTOR_HPP
