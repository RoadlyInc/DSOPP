
#ifndef DSOPP_SRC_FEATURES_CAMERA_TRACKING_FEATURES_FRAME_HPP_
#define DSOPP_SRC_FEATURES_CAMERA_TRACKING_FEATURES_FRAME_HPP_

#include <memory>
#include <vector>

namespace dsopp {
namespace features {
class TrackingFeature;
/**
 * \brief TrackingFeaturesFrame is a container for tracking features
 *
 * TrackingFeaturesFrame is a container for tracking features from one frame data.
 */
class TrackingFeaturesFrame {
 public:
  /**
   * creates a features frame from a vector of tracking features.
   * @param tracking_features vector of tracking features
   */
  explicit TrackingFeaturesFrame(std::vector<TrackingFeature> &&tracking_features);

  ~TrackingFeaturesFrame();
  /**
   * @return tracking features
   */
  const std::vector<TrackingFeature> &features() const;

 private:
  /** vector of tracking features */
  std::vector<TrackingFeature> features_;
};
}  // namespace features
}  // namespace dsopp

#endif  // DSOPP_SRC_FEATURES_CAMERA_TRACKING_FEATURES_FRAME_HPP_
