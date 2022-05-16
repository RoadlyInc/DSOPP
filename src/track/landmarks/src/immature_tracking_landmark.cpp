#include "track/landmarks/immature_tracking_landmark.hpp"

#include "track/landmarks/active_tracking_landmark.hpp"

namespace dsopp {
namespace track {
namespace landmarks {

ImmatureTrackingLandmark::ImmatureTrackingLandmark(const Eigen::Vector3<Precision>& direction,
                                                   const Eigen::Vector2<Precision>& projection,
                                                   const Eigen::Vector<Precision, Pattern::kSize>& patch,
                                                   const Eigen::Vector2<Precision>& gradient)
    : TrackingLandmarkWithPatch(direction, projection, patch), gradient_(gradient) {}

Precision ImmatureTrackingLandmark::idepthMin() const { return idepth_min_; }

void ImmatureTrackingLandmark::setIdepthMin(Precision idepth) { idepth_min_ = idepth; }

Precision ImmatureTrackingLandmark::idepthMax() const { return idepth_max_; }

void ImmatureTrackingLandmark::setIdepthMax(Precision idepth) { idepth_max_ = idepth; }

Precision ImmatureTrackingLandmark::idepth() const { return idepth_max_ * 0.5_p + idepth_min_ * 0.5_p; }

void ImmatureTrackingLandmark::setStatus(ImmatureStatus status) {
  if (status == ImmatureStatus::kGood) {
    traced_ = true;
  }
  status_ = status;
}

ImmatureStatus ImmatureTrackingLandmark::status() const { return status_; }

const Eigen::Vector2<Precision>& ImmatureTrackingLandmark::gradient() const { return gradient_; }

void ImmatureTrackingLandmark::setSearchPixelInterval(Precision search_pixel_interval) {
  search_pixel_interval_ = search_pixel_interval;
}

void ImmatureTrackingLandmark::setUniqueness(Precision uniqueness, bool force_set) {
  if (force_set or uniqueness < uniqueness_) {
    uniqueness_ = uniqueness;
  }
}

bool ImmatureTrackingLandmark::readyForActivation() const {
  Precision kMaxSearchPixelInterval = 8;
  Precision kMinUniqueness = 3;
  return ((status_ == ImmatureStatus::kGood or status_ == ImmatureStatus::kSkipped or
           status_ == ImmatureStatus::kIllConditioned or status_ == ImmatureStatus::kOutOfBoundary) and
          (search_pixel_interval_ < kMaxSearchPixelInterval) and (uniqueness_ > kMinUniqueness) and (idepth() > 0));
}

bool ImmatureTrackingLandmark::isTraced() const { return traced_; }
}  // namespace landmarks
}  // namespace track
}  // namespace dsopp
