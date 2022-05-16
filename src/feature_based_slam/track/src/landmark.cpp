#include "feature_based_slam/track/landmark.hpp"

#include "feature_based_slam/features/distinct_feature.hpp"

namespace dsopp {
namespace feature_based_slam {
namespace track {

Landmark::Landmark(const Eigen::Vector3<Precision> &position) : initialized_(true), position_(position) {}

Landmark::Landmark() { initialized_ = false; }

const Eigen::Vector3<Precision> *Landmark::position() const { return initialized_ ? &position_ : nullptr; }

void Landmark::setPosition(const Eigen::Vector3<Precision> &position) {
  initialized_ = true;
  position_ = position;
}

void Landmark::addProjection(size_t frame_id, const features::DistinctFeature &feature) {
  projections_.insert({frame_id, feature});
}

const features::DistinctFeature *Landmark::projection(size_t frame_id) const {
  return projections_.contains(frame_id) ? &projections_.at(frame_id) : nullptr;
}

void Landmark::removeProjection(size_t frame_id) {
  projections_.erase(frame_id);
  initialized_ = initialized_ && projections_.size() > 1;
}

}  // namespace track
}  // namespace feature_based_slam
}  // namespace dsopp
