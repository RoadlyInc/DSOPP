#ifndef DSOPP_SRC_FEATURES_CAMERA_EIGEN_TRACKING_FEATURES_EXTRACTOR_HPP
#define DSOPP_SRC_FEATURES_CAMERA_EIGEN_TRACKING_FEATURES_EXTRACTOR_HPP

#include "features/camera/tracking_features_extractor.hpp"

#include <memory>
#include <string>

#include "common/settings.hpp"

namespace dsopp {
namespace features {
class TrackingFeaturesFrame;
/**
 * \brief TrackingFeatureExtractor extract feature points for tracking from frame data.
 *
 * Object of this class extract feature points from frame data for future
 * calculating transformation between images.
 */
class EigenTrackingFeaturesExtractor : public TrackingFeaturesExtractor {
 public:
  /**
   * Creates a tracking feature extractor.
   * @param point_density_for_detector desired number of the feature points
   */
  explicit EigenTrackingFeaturesExtractor(const Precision point_density_for_detector = 1500);

  std::unique_ptr<TrackingFeaturesFrame> extract(cv::Mat image, const sensors::calibration::CameraMask &mask) override;
};

}  // namespace features
}  // namespace dsopp

#endif  // DSOPP_SRC_FEATURES_CAMERA_EIGEN_TRACKING_FEATURES_EXTRACTOR_HPP
