#include "feature_based_slam/features/distinct_features_frame.hpp"
#include "feature_based_slam/features/distinct_feature.hpp"

#include <string>
namespace dsopp {
namespace feature_based_slam {
namespace features {
DistinctFeaturesFrame::DistinctFeaturesFrame(std::vector<DistinctFeature> &&distinct_features, cv::Mat descriptors)
    : distinct_features_(std::move(distinct_features)), descriptors_(descriptors) {}

const std::vector<DistinctFeature> &DistinctFeaturesFrame::features() const { return distinct_features_; }

bool DistinctFeaturesFrame::hasDescriptors() const { return !descriptors_.empty(); }

const cv::Mat DistinctFeaturesFrame::descriptors() const { return descriptors_; }

DistinctFeaturesFrame::~DistinctFeaturesFrame() = default;
}  // namespace features
}  // namespace feature_based_slam
}  // namespace dsopp
