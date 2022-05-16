#include "features/camera/tracking_feature.hpp"

namespace dsopp {
namespace features {
TrackingFeature::TrackingFeature(const Eigen::Vector2<Precision> &coordinates) : coordinates_(coordinates) {}
const Eigen::Vector2<Precision> &TrackingFeature::coordinates() const { return coordinates_; }
}  // namespace features
}  // namespace dsopp
