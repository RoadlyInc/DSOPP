#ifndef DSOPP_SRC_TRACKER_KEYFRAME_STRATEGY_MEAN_SQUARE_OPTICAL_FLOW_KEYFRAME_STRATEGY_HPP_
#define DSOPP_SRC_TRACKER_KEYFRAME_STRATEGY_MEAN_SQUARE_OPTICAL_FLOW_KEYFRAME_STRATEGY_HPP_

#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"
#include "tracker/keyframe_strategy/tracker_keyframe_strategy.hpp"

namespace dsopp {
namespace tracker {
namespace keyframe_strategy {
/**
 * \brief Keyframe strategy that creates keyframes according to change of mean square optical flow and rmse in energy.
 */

// TODO: rename it
template <energy::motion::Motion Motion>
class MeanSquareOpticalFlowAndRmseKeyframeStrategy : public TrackerKeyframeStrategy<Motion> {
 public:
  /**
   * creates MeanSquareOpticalFlowAndRmseKeyframeStrategy with a given factor for threshold
   * @param factor the larger the more keyframes are created (e.g., 2 = double number of keyframes)
   */
  explicit MeanSquareOpticalFlowAndRmseKeyframeStrategy(double factor);
  bool needNewKeyframe(const track::ActiveOdometryTrack<Motion> &track,
                       const track::SLAMInternalTrackingFrame<Motion> &frame) override;
  ~MeanSquareOpticalFlowAndRmseKeyframeStrategy() override;

 private:
  /** Weight for mean square optical flow term */
  const double kMaxShiftWeight_ = 4.5;
  /** Weight for mean square optical flow without rotation term */
  const double kMaxShiftWithoutRotationWeight_ = 9;
  /** Weight for max affine brightness term */
  const double kMaxAffineBrightnessWeight_ = 2;
  /** Threshold to create a new keyframe  */
  const double kThreshold_ = 1;
  /** Excess energy relative to the keyframe in kMaxExcessOffEnergy times creates a new keyframe */
  const double kMaxExcessOffEnergy_ = 4.;
  /** factor for threshold, the larger the more keyframes are created (e.g., 2 = double number of keyframes) */
  double factor_;
  /** mean energy of the first frame after a keyframe */
  double rmse_;
};

}  // namespace keyframe_strategy
}  // namespace tracker
}  // namespace dsopp

#endif  // DSOPP_SRC_TRACKER_KEYFRAME_STRATEGY_MEAN_SQUARE_OPTICAL_FLOW_KEYFRAME_STRATEGY_HPP_
