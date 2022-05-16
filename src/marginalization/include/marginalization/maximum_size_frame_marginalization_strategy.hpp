#ifndef DSOPP_MAXIMUM_SIZE_FRAME_MARGINALIZATION_STRATEGY_HPP
#define DSOPP_MAXIMUM_SIZE_FRAME_MARGINALIZATION_STRATEGY_HPP
#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"
#include "marginalization/frame_marginalization_strategy.hpp"

namespace dsopp {
namespace track {
template <energy::motion::Motion Motion>
class ActiveOdometryTrack;
}
namespace marginalization {
/**
 * simple marginalization strategy: keeps only fixed number of active keyframes
 */
template <energy::motion::Motion Motion>
class MaximumSizeFrameMarginalizationStrategy : public FrameMarginalizationStrategy<Motion> {
 public:
  /**
   *
   * @param maximum_size maximum number of active keyframes
   */
  MaximumSizeFrameMarginalizationStrategy(size_t maximum_size);

  /** method to override
   *
   * @param[out] track track to be marginalized
   */
  void marginalize(track::ActiveOdometryTrack<Motion>& track) override;
  ~MaximumSizeFrameMarginalizationStrategy() override;

 private:
  /** maximum number of active frames in track */
  size_t maximum_size_;
};

}  // namespace marginalization
}  // namespace dsopp

#endif  // DSOPP_MAXIMUM_SIZE_FRAME_MARGINALIZATION_STRATEGY_HPP
