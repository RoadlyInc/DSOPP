#include "feature_based_slam/tracker/tracker.hpp"

#include <glog/logging.h>

namespace dsopp {
namespace feature_based_slam {
namespace tracker {

Tracker::Tracker(
    const size_t sensor_id, std::unique_ptr<features::DistinctFeaturesExtractor> &&feature_extractor,
    std::function<std::unique_ptr<Initializer>(track::Track &, const features::DistinctFeaturesExtractor &)>
        &initializer_fabric,
    const Options &options)
    : sensor_id_(sensor_id), feature_extractor_(std::move(feature_extractor)), options_(options) {
  initializer_ = initializer_fabric(track_, *feature_extractor_);
}

void Tracker::clear() { initializer_->clear(); }

bool Tracker::initialized() { return initializer_->initialized(); }

size_t Tracker::evaluatedFramesSize() const { return track_.frames.size(); }

const energy::motion::SE3<Precision> &Tracker::tWorldAgent(size_t idx) const {
  CHECK_LT(idx, track_.frames.size());
  return track_.frames[idx].t_world_agent;
}

time Tracker::timestamp(size_t idx) const {
  CHECK_LT(idx, track_.frames.size());
  return track_.frames[idx].timestamp;
}

bool Tracker::isInitialized(size_t idx) const {
  CHECK_LT(idx, track_.frames.size());
  return track_.frames[idx].initialized;
}

Tracker::~Tracker() = default;

}  // namespace tracker
}  // namespace feature_based_slam
}  // namespace dsopp
