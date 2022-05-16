#ifndef DSOPP_SRC_TRACKER_KEYFRAME_STRATEGY_FREQUENCY_KEYFRAME_STRATEGY_HPP_
#define DSOPP_SRC_TRACKER_KEYFRAME_STRATEGY_FREQUENCY_KEYFRAME_STRATEGY_HPP_

#include "common/time/time.hpp"
#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"
#include "tracker/keyframe_strategy/tracker_keyframe_strategy.hpp"

namespace dsopp {
namespace tracker {
namespace keyframe_strategy {
/**
 * \brief Keyframe strategy that creates the given number of keyframes per second.
 */
template <energy::motion::Motion Motion>
class FrequencyKeyframeStrategy : public TrackerKeyframeStrategy<Motion> {
 public:
  /**
   * creates FrequencyKeyframeStrategy with a given frequency
   * @param frequency number of keyframes per second to be created
   */
  explicit FrequencyKeyframeStrategy(double frequency);

  bool needNewKeyframe(const track::ActiveOdometryTrack<Motion> &track,
                       const track::SLAMInternalTrackingFrame<Motion> &frame) override;

  ~FrequencyKeyframeStrategy() override;

 private:
  /** time between two keyframes in milliseconds - number inverse to frequency */
  long period_ms_;
};

}  // namespace keyframe_strategy
}  // namespace tracker
}  // namespace dsopp

#endif  // DSOPP_SRC_TRACKER_KEYFRAME_STRATEGY_FREQUENCY_KEYFRAME_STRATEGY_HPP_
