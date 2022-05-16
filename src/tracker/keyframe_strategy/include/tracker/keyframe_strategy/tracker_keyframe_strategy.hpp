#ifndef DSOPP_SRC_TRACKER_KEYFRAME_STRATEGY_TRACKER_KEYFRAME_STRATEGY_HPP_
#define DSOPP_SRC_TRACKER_KEYFRAME_STRATEGY_TRACKER_KEYFRAME_STRATEGY_HPP_

#include <memory>

#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"

namespace dsopp {
namespace track {
template <energy::motion::Motion Motion>
class SLAMInternalTrackingFrame;
template <energy::motion::Motion Motion>
class ActiveOdometryTrack;
}  // namespace track
namespace tracker {
namespace keyframe_strategy {
/**
 * \brief TrackerKeyframeStrategy interface.
 *
 * Each strategy for selecting a new keyframe during tracking should implement this interface.
 */
template <energy::motion::Motion Motion>
class TrackerKeyframeStrategy {
 public:
  /**
   * method to check if the new frame must be the keyframe
   * @param track track with keyframes
   * @param frame new frame to check
   * @return true if the new frame must be the keyframe and false otherwise
   */
  virtual bool needNewKeyframe(const track::ActiveOdometryTrack<Motion> &track,
                               const track::SLAMInternalTrackingFrame<Motion> &frame) = 0;
  virtual ~TrackerKeyframeStrategy() = 0;
};

}  // namespace keyframe_strategy
}  // namespace tracker
}  // namespace dsopp

#endif  // DSOPP_SRC_TRACKER_KEYFRAME_STRATEGY_TRACKER_KEYFRAME_STRATEGY_HPP_
