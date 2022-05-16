#ifndef DSOPP_SRC_TRACK_LANDMARKS_IMMATURE_TRACKING_LANDMARK_HPP_
#define DSOPP_SRC_TRACK_LANDMARKS_IMMATURE_TRACKING_LANDMARK_HPP_

#include "common/pattern/pattern.hpp"
#include "common/settings.hpp"
#include "track/landmarks/tracking_landmark_with_patch.hpp"

namespace dsopp {
namespace track {
namespace landmarks {

class ActiveTrackingLandmark;
/** status of the immature point */
enum struct ImmatureStatus : uint8_t {
  kGood = 0,       /**< traced well and good. */
  kOutOfBoundary,  /**< out of boundary: end tracking & marginalize! */
  kOutlier,        /**< energy too high: if happens again: outlier! */
  kSkipped,        /**< traced well and good (but not actually traced). */
  kIllConditioned, /**< not traced because of ill condition. */
  kUninitialized,  /**< not even traced once. */
  kDelete          /**< will be deleted during marginalization.; */
};
/**
 * class to store immature 3d landmark. Immature landmark has no idepth, only minimum and maximum limits.
 */
class ImmatureTrackingLandmark : public TrackingLandmarkWithPatch {
 public:
  /**
   * create immature point (point with uncertainty in the depth)
   * @param direction direction vector
   * @param projection projection point
   * @param patch patch on the corresponding image
   * @param gradient gradient of the patch
   */
  ImmatureTrackingLandmark(const Eigen::Vector3<Precision>& direction, const Eigen::Vector2<Precision>& projection,
                           const Eigen::Vector<Precision, Pattern::kSize>& patch,
                           const Eigen::Vector2<Precision>& gradient);
  /**
   * @return minimum idepth limit
   */
  Precision idepthMin() const;
  /**
   * set minimum idepth limit
   * @param idepth new minimum idepth limit
   */
  void setIdepthMin(Precision idepth);
  /**
   * @return maximum idepth limit
   */
  Precision idepthMax() const;
  /**
   * set maximum idepth limit
   * @param idepth new maximum idepth limit
   */
  void setIdepthMax(Precision idepth);
  /**
   * @return average idepth
   */
  Precision idepth() const;
  /**
   * set immature status
   * @param status new immature status
   */
  void setStatus(ImmatureStatus status);
  /**
   * @return immature status
   */
  ImmatureStatus status() const;
  /**
   * @return gradient of the patch
   */
  const Eigen::Vector2<Precision>& gradient() const;
  /**
   * @param search_pixel_interval the interval at which the point was searched for the last time
   */
  void setSearchPixelInterval(Precision search_pixel_interval);
  /**
   * @param uniqueness of a point on the epipolar line.
   * @param force_set force set new value of uniqueness
   */
  void setUniqueness(Precision uniqueness, bool force_set);
  /**
   * @return true if landmark ready to activate and false otherwise
   */
  bool readyForActivation() const;
  /**
   * @return true if the landmark is traced on successfully at least once and false otherwise
   */
  bool isTraced() const;

 private:
  /** minimum idepth limit */
  Precision idepth_min_ = 0;
  /** maximum idepth limit */
  Precision idepth_max_ = 1. / 0.001;
  /** uniqueness of a point on the epipolar line. If it is close to unity, then there is another
   * point with the same energy on the epipolar line. */
  Precision uniqueness_ = std::numeric_limits<Precision>::max();
  /** the interval at which the point was searched for the last time. */
  Precision search_pixel_interval_ = std::numeric_limits<Precision>::max();
  /** status of the immature point */
  ImmatureStatus status_ = ImmatureStatus::kUninitialized;
  /** summary gradient of the patch */
  const Eigen::Vector2<Precision> gradient_;
  /** true if the landmark is traced on successfully at least once */
  bool traced_ = false;
};
}  // namespace landmarks
}  // namespace track
}  // namespace dsopp

#endif  // DSOPP_SRC_TRACK_LANDMARKS_IMMATURE_TRACKING_LANDMARK_HPP_
