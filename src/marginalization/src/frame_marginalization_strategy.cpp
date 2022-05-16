#include "marginalization/frame_marginalization_strategy.hpp"

#include "energy/motion/se3_motion.hpp"

namespace dsopp {
namespace marginalization {

template <energy::motion::Motion Motion>
FrameMarginalizationStrategy<Motion>::~FrameMarginalizationStrategy() = default;

template class FrameMarginalizationStrategy<energy::motion::SE3<Precision>>;

}  // namespace marginalization

}  // namespace dsopp
