
#ifndef DSOPP_TRACKING_LANDMARK_BASE_HPP
#define DSOPP_TRACKING_LANDMARK_BASE_HPP

#include <Eigen/Dense>

#include "common/settings.hpp"

namespace dsopp {
namespace track {
namespace landmarks {
/**
 * Base class for all 3d landmarks (Immature, Activated, Marginalized, Result).
 */
class TrackingLandmarkBase {
 public:
  /**
   * create point
   * @param direction direction vector
   * @param projection projection point
   */
  TrackingLandmarkBase(const Eigen::Vector3<Precision>& direction, const Eigen::Vector2<Precision>& projection);
  /**
   * @return direction of the point
   */
  const Eigen::Vector3<Precision>& direction() const;
  /**
   * @return projection of the point to image plane
   */
  const Eigen::Vector2<Precision>& projection() const;
  /**
   * @return true if landmark is marked an outlier
   */
  bool isOutlier() const;
  /**
   * marks landmark as outlier
   */
  void markOutlier();

 protected:
  /** vector representing direction from 0 of the camera to the point. Lies on the focal plane (z = 1)*/
  const Eigen::Vector3<Precision> direction_;
  /** 2D vector representing projection to image plane */
  const Eigen::Vector2<Precision> projection_;
  /** true if landmark is marked an outlier */
  bool is_outlier_ = false;
};
}  // namespace landmarks
}  // namespace track
}  // namespace dsopp

#endif  // DSOPP_TRACKING_LANDMARK_BASE_HPP
