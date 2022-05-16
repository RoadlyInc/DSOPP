#include "tracker/keyframe_strategy//tracker_keyframe_strategy.hpp"
#include "common/settings.hpp"

namespace dsopp {
namespace tracker {
namespace keyframe_strategy {
template <energy::motion::Motion Motion>
TrackerKeyframeStrategy<Motion>::~TrackerKeyframeStrategy() = default;

template class TrackerKeyframeStrategy<energy::motion::SE3<Precision>>;

}  // namespace keyframe_strategy
}  // namespace tracker
}  // namespace dsopp
