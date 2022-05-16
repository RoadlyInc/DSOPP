#include "track/landmarks/tracking_landmark.hpp"

namespace dsopp {
namespace track {
namespace landmarks {
TrackingLandmark::TrackingLandmark(const Eigen::Vector3<Precision>& direction,
                                   const Eigen::Vector2<Precision>& projection, Precision idepth,
                                   Precision idepth_variance, Precision relative_baseline, size_t semantic_type_id)
    : TrackingLandmarkBase(direction, projection),
      idepth_(idepth),
      idepth_variance_(idepth_variance),
      relative_baseline_(relative_baseline),
      semantic_type_id_(semantic_type_id) {}

TrackingLandmark::TrackingLandmark(const proto::Landmark& proto)
    : TrackingLandmarkBase(
          Eigen::Vector3d(proto.direction_x(), proto.direction_y(), proto.direction_z()).cast<Precision>(),
          Eigen::Vector2d(proto.projection_x(), proto.projection_y()).cast<Precision>()),
      idepth_(static_cast<Precision>(proto.idepth())),
      idepth_variance_(static_cast<Precision>(proto.idepth_variance())),
      relative_baseline_(static_cast<Precision>(proto.relative_baseline())),
      semantic_type_id_(proto.semantic_type_id()) {}

Precision TrackingLandmark::idepth() const { return idepth_; }

Precision TrackingLandmark::idepthVariance() const { return idepth_variance_; }

Precision TrackingLandmark::relativeBaseline() const { return relative_baseline_; }

proto::Landmark TrackingLandmark::proto() const {
  proto::Landmark proto;
  proto.set_direction_x(direction_[0]);
  proto.set_direction_y(direction_[1]);
  proto.set_direction_z(direction_[2]);
  proto.set_projection_x(projection_[0]);
  proto.set_projection_y(projection_[1]);
  proto.set_idepth(idepth_);
  proto.set_idepth_variance(idepth_variance_);
  proto.set_relative_baseline(relative_baseline_);
  proto.set_semantic_type_id(semantic_type_id_);
  return proto;
}

size_t TrackingLandmark::semanticTypeId() const { return semantic_type_id_; }

}  // namespace landmarks
}  // namespace track
}  // namespace dsopp
