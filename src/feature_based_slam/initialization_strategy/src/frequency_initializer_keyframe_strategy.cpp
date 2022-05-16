#include "feature_based_slam/initialization_strategy/frequency_initializer_keyframe_strategy.hpp"

namespace dsopp::feature_based_slam::initialization_strategy {

FrequencyInitializerKeyframeStrategy::FrequencyInitializerKeyframeStrategy(Precision frequency)
    : period_ms_(static_cast<long>(1.0 / frequency * 1000)) {}

FrameStatus FrequencyInitializerKeyframeStrategy::needNewFrame(const dsopp::features::CameraFeatures &new_frame,
                                                               const std::deque<track::Frame> &previous_frames,
                                                               const size_t, const size_t) {
  if (previous_frames.empty()) return FrameStatus::kKeepFrame;

  auto first_timestamp = previous_frames.front().timestamp;
  long delta_t_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(new_frame.timestamp() - first_timestamp).count();
  if (delta_t_ms >= period_ms_) {
    return FrameStatus::kFinish;
  }
  return FrameStatus::kKeepFrame;
}

bool FrequencyInitializerKeyframeStrategy::managesStandstill() { return false; }

FrequencyInitializerKeyframeStrategy::~FrequencyInitializerKeyframeStrategy() = default;

}  // namespace dsopp::feature_based_slam::initialization_strategy
