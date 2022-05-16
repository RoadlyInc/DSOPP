#ifndef DSOPP_SRC_TRACKER_KEYFRAME_FABRIC_HPP_
#define DSOPP_SRC_TRACKER_KEYFRAME_FABRIC_HPP_

#include <any>
#include <map>
#include <memory>

#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"

namespace dsopp {
namespace tracker {
namespace keyframe_strategy {
template <energy::motion::Motion Motion>
class InitializerKeyframeStrategy;
template <energy::motion::Motion Motion>
class TrackerKeyframeStrategy;

/**
 * creates TrackerKeyframeStrategy with a given configuration
 * @param parameters parameters specific for the TrackerKeyframeStrategy
 * @return created keyframe strategy or nullptr on failure
 */
template <energy::motion::Motion Motion>
std::unique_ptr<TrackerKeyframeStrategy<Motion>> createTrackerKeyframeStrategy(
    const std::map<std::string, std::any> &parameters);

}  // namespace keyframe_strategy
}  // namespace tracker
}  // namespace dsopp

#endif  // DSOPP_SRC_TRACKER_KEYFRAME_FABRIC_HPP_
