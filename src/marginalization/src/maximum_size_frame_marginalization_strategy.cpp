#include "marginalization/maximum_size_frame_marginalization_strategy.hpp"

#include "energy/motion/se3_motion.hpp"

#include "track/active_odometry_track.hpp"

#include <numeric>
namespace dsopp {
namespace marginalization {

template <energy::motion::Motion Motion>
MaximumSizeFrameMarginalizationStrategy<Motion>::MaximumSizeFrameMarginalizationStrategy(size_t maximum_size)
    : maximum_size_(maximum_size) {}

template <energy::motion::Motion Motion>
void MaximumSizeFrameMarginalizationStrategy<Motion>::marginalize(track::ActiveOdometryTrack<Motion>& track) {
  while (track.activeFrames().size() > maximum_size_) {
    track.marginalizeFrame(0);
  }
}
template <energy::motion::Motion Motion>
MaximumSizeFrameMarginalizationStrategy<Motion>::~MaximumSizeFrameMarginalizationStrategy() = default;

template class MaximumSizeFrameMarginalizationStrategy<energy::motion::SE3<Precision>>;

}  // namespace marginalization
}  // namespace dsopp
