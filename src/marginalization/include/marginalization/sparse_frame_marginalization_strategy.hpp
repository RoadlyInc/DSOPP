#ifndef DSOPP_SPARSE_FRAME_MARGINALIZATION_STRATEGY_HPP
#define DSOPP_SPARSE_FRAME_MARGINALIZATION_STRATEGY_HPP
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
 * This marginalization strategy maintains the number of active keyframes between the minimum and maximum values.
 * Active Keyframes are kept in a sparse form. Frames with large numbers of marginalized landmarks marginalized.
 * @tparam N pattern patch size
 */
template <energy::motion::Motion Motion>
class SparseFrameMarginalizationStrategy : public FrameMarginalizationStrategy<Motion> {
 public:
  /**
   *
   * @param minimum_size min number of active keyframes
   * @param maximum_size max number of active keyframes
   * @param maximum_number_of_marginalized maximum number of marginalized landmarks in the frame
   */
  SparseFrameMarginalizationStrategy(size_t minimum_size, size_t maximum_size, double maximum_number_of_marginalized);

  /** method to override
   *
   * @param[out] track track to be marginalized
   */
  void marginalize(track::ActiveOdometryTrack<Motion>& track) override;
  ~SparseFrameMarginalizationStrategy() override;

 private:
  /** minimum number of active frames in track */
  size_t minimum_size_;
  /** maximum number of active frames in track */
  size_t maximum_size_;
  /** maximum number of marginalized landmarks in the frame */
  double maximum_number_of_marginalized_;
};
}  // namespace marginalization
}  // namespace dsopp

#endif  // DSOPP_SPARSE_FRAME_MARGINALIZATION_STRATEGY_HPP
