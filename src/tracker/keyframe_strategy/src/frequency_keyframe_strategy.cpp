#include "tracker/keyframe_strategy//frequency_keyframe_strategy.hpp"

#include "sensor/synchronized_frame.hpp"
#include "track/active_odometry_track.hpp"
#include "track/frames/active_keyframe.hpp"

namespace dsopp {
namespace tracker {
namespace keyframe_strategy {
namespace {
template <energy::motion::Motion Motion, class FrameType>
bool isKeyframe(const track::ActiveOdometryTrack<Motion> &track, const FrameType &frame, long period_ms) {
  if (track.frames().empty()) {
    return true;
  } else {
    auto last_kf_timestamp = track.frames().back()->timestamp();
    long delta_t_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(frame.timestamp() - last_kf_timestamp).count();
    return delta_t_ms >= period_ms;
  };
}
}  // namespace
template <energy::motion::Motion Motion>
FrequencyKeyframeStrategy<Motion>::FrequencyKeyframeStrategy(double frequency)
    : period_ms_(static_cast<long>(1.0 / frequency * 1000)) {}

template <energy::motion::Motion Motion>
bool FrequencyKeyframeStrategy<Motion>::needNewKeyframe(const track::ActiveOdometryTrack<Motion> &track,
                                                        const track::SLAMInternalTrackingFrame<Motion> &frame) {
  return isKeyframe(track, frame, period_ms_);
}
template <energy::motion::Motion Motion>
FrequencyKeyframeStrategy<Motion>::~FrequencyKeyframeStrategy() = default;

template class FrequencyKeyframeStrategy<energy::motion::SE3<Precision>>;

}  // namespace keyframe_strategy
}  // namespace tracker
}  // namespace dsopp
