#ifndef DSOPP_SRC_FEATURES_CAMERA_TRACKING_FEATURE_HPP_
#define DSOPP_SRC_FEATURES_CAMERA_TRACKING_FEATURE_HPP_

#include <Eigen/Dense>

#include "common/settings.hpp"

namespace dsopp {
namespace features {
/**
 * \brief TrackingFeature represent an identifiable feature for tracking.
 *
 * Object of this class stores coordinates the tracking feature.
 */
class TrackingFeature {
 public:
  /**
   * creates a tracking feature from coordinates.
   * @param coordinates coordinates of the tracking feature
   */
  TrackingFeature(const Eigen::Matrix<Precision, 2, 1> &coordinates);
  /**
   * @return coordinates of the point
   */
  const Eigen::Matrix<Precision, 2, 1> &coordinates() const;

 protected:
  /** feature coordinates*/
  Eigen::Matrix<Precision, 2, 1> coordinates_;
};
}  // namespace features
}  // namespace dsopp

#endif  // DSOPP_SRC_FEATURES_CAMERA_TRACKING_FEATURE_HPP_
