#include "features/camera/tracking_features_frame.hpp"
#include "features/camera/tracking_feature.hpp"

#include <string>
namespace dsopp {
namespace features {
TrackingFeaturesFrame::TrackingFeaturesFrame(std::vector<TrackingFeature> &&tracking_features)
    : features_(std::move(tracking_features)) {}
const std::vector<TrackingFeature> &TrackingFeaturesFrame::features() const { return features_; }
TrackingFeaturesFrame::~TrackingFeaturesFrame() = default;
}  // namespace features
}  // namespace dsopp
