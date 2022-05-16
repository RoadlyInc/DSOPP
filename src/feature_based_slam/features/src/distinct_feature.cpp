#include "feature_based_slam/features//distinct_feature.hpp"

namespace dsopp {
namespace feature_based_slam {
namespace features {
DistinctFeature::DistinctFeature(const Eigen::Vector<Precision, 2> &coordinates) : coordinates_(coordinates) {}

const Eigen::Vector<Precision, 2> &DistinctFeature::coordinates() const { return coordinates_; }

DistinctFeature::~DistinctFeature() = default;
}  // namespace features
}  // namespace feature_based_slam
}  // namespace dsopp
