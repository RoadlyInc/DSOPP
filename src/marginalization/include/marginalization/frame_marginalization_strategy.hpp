#ifndef DSOPP_FRAME_MARGINALIZATION_STRATEGY_HPP
#define DSOPP_FRAME_MARGINALIZATION_STRATEGY_HPP
#include <cstddef>
#include <vector>

#include "common/settings.hpp"
#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"

namespace dsopp {
namespace track {
template <energy::motion::Motion Motion>
class ActiveOdometryTrack;
}
namespace marginalization {
/**
 * interface for marginalization strategies
 * @tparam Motion Motion type
 */
template <energy::motion::Motion Motion>
class FrameMarginalizationStrategy {
 public:
  /**
   * method decides which frames to marginalize
   * @param track track to be marginalized
   */
  virtual void marginalize(track::ActiveOdometryTrack<Motion>& track) = 0;
  virtual ~FrameMarginalizationStrategy() = 0;
};

}  // namespace marginalization
}  // namespace dsopp
#endif  // DSOPP_FRAME_MARGINALIZATION_STRATEGY_HPP
