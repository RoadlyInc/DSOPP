#include "track/landmarks/tracking_landmark_with_patch.hpp"

namespace dsopp {
namespace track {
namespace landmarks {
TrackingLandmarkWithPatch::TrackingLandmarkWithPatch(const Eigen::Vector3<Precision>& direction,
                                                     const Eigen::Vector2<Precision>& projection,
                                                     const Eigen::Vector<Precision, Pattern::kSize>& patch)
    : TrackingLandmarkBase(direction, projection),
      patch_(std::make_unique<const Eigen::Vector<Precision, Pattern::kSize>>(patch)) {}

const Eigen::Vector<Precision, Pattern::kSize>& TrackingLandmarkWithPatch::patch() const { return *patch_; }

}  // namespace landmarks
}  // namespace track
}  // namespace dsopp
