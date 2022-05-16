#ifndef DSOPP_SRC_TRACK_LANDMARKS_TRACKING_LANDMARK_WITH_PATCH_HPP_
#define DSOPP_SRC_TRACK_LANDMARKS_TRACKING_LANDMARK_WITH_PATCH_HPP_

#include "common/patch/patch.hpp"
#include "common/pattern/pattern.hpp"
#include "common/settings.hpp"
#include "track/landmarks/tracking_landmark_base.hpp"

#include <memory>

namespace dsopp::track::landmarks {
/**
 * landmark with the correspondent patches
 */
class TrackingLandmarkWithPatch : public TrackingLandmarkBase {
 public:
  /**
   * create landmark point with patches
   * @param direction direction vector
   * @param projection projection point
   * @param patch patch on the corresponding image
   */
  TrackingLandmarkWithPatch(const Eigen::Vector3<Precision>& direction, const Eigen::Vector2<Precision>& projection,
                            const Eigen::Vector<Precision, Pattern::kSize>& patch);
  /**
   * @return patch on the image
   */
  const Eigen::Vector<Precision, Pattern::kSize>& patch() const;

 protected:
  /** patch of the landmark */
  std::unique_ptr<const Eigen::Vector<Precision, Pattern::kSize>> patch_;
};
}  // namespace dsopp::track::landmarks

#endif  // DSOPP_SRC_TRACK_LANDMARKS_TRACKING_LANDMARK_WITH_PATCH_HPP_
