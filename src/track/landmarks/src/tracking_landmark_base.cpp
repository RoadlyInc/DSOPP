
#include "track/landmarks/tracking_landmark_base.hpp"

namespace dsopp {
namespace track {
namespace landmarks {
TrackingLandmarkBase::TrackingLandmarkBase(const Eigen::Vector3<Precision>& direction,
                                           const Eigen::Vector2<Precision>& projection)
    : direction_(direction), projection_(projection) {}
const Eigen::Vector3<Precision>& TrackingLandmarkBase::direction() const { return direction_; }
const Eigen::Vector2<Precision>& TrackingLandmarkBase::projection() const { return projection_; }

bool TrackingLandmarkBase::isOutlier() const { return is_outlier_; }
void TrackingLandmarkBase::markOutlier() { is_outlier_ = true; }

}  // namespace landmarks
}  // namespace track
}  // namespace dsopp
