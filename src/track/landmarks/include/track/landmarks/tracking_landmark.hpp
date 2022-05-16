#ifndef DSOPP_TRACKING_LANDMARK_HPP
#define DSOPP_TRACKING_LANDMARK_HPP

#include "common/settings.hpp"
#include "track/landmarks/tracking_landmark_base.hpp"

#include "landmark.pb.h"
namespace dsopp {
namespace track {
namespace landmarks {
/**
 * class to store result 3d landmark. result landmark has a fixed idepth and doesn't hold any data for optimization.
 */
class TrackingLandmark : public TrackingLandmarkBase {
 public:
  /**
   * create result point
   * @param direction direction vector
   * @param projection projection point
   * @param idepth idepth
   * @param idepth_variance idepth variance
   * @param relative_baseline relative baseline (ratio between baseline in optimization window and depth)
   * @param semantic_type_id semantic class
   */
  TrackingLandmark(const Eigen::Vector3<Precision>& direction, const Eigen::Vector2<Precision>& projection,
                   Precision idepth, Precision idepth_variance, Precision relative_baseline,
                   size_t semantic_type_id = 0);
  /**
   * creates result landmark from the protobuf container
   * @param proto protobuf container
   */
  explicit TrackingLandmark(const proto::Landmark& proto);
  /**
   * creates protobuf container from the object
   * @return protobuf container
   */
  proto::Landmark proto() const;
  /**
   * @return idepth
   */
  Precision idepth() const;
  /**
   * @return idepth variance
   */
  Precision idepthVariance() const;
  /**
   * @return relative baseline
   */
  Precision relativeBaseline() const;

  /**
   * get semantic type
   * @return semantic type
   */
  size_t semanticTypeId() const;

 private:
  /** idepth */
  const Precision idepth_;
  /** idepth variance*/
  const Precision idepth_variance_;
  /** relative baseline (ratio between baseline in optimization window and depth) */
  const Precision relative_baseline_;
  /** semantic type of landmark */
  const size_t semantic_type_id_ = 0;
};
}  // namespace landmarks
}  // namespace track
}  // namespace dsopp

#endif  // DSOPP_TRACKING_LANDMARK_HPP
