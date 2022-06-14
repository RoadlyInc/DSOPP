#include "tracker/keyframe_strategy//mean_square_optical_flow_and_rmse_keyframe_strategy.hpp"

#include "track/active_odometry_track.hpp"
#include "track/frames/active_keyframe.hpp"

namespace dsopp {
namespace tracker {
namespace keyframe_strategy {
template <energy::motion::Motion Motion>
MeanSquareOpticalFlowAndRmseKeyframeStrategy<Motion>::MeanSquareOpticalFlowAndRmseKeyframeStrategy(double factor)
    : factor_(factor), rmse_(-1) {}

template <energy::motion::Motion Motion>
bool MeanSquareOpticalFlowAndRmseKeyframeStrategy<Motion>::needNewKeyframe(
    const track::ActiveOdometryTrack<Motion> &track, const track::SLAMInternalTrackingFrame<Motion> &frame) {
  const auto &last_kf_affine_brightness = track.activeFrames().back()->affineBrightness();
  const auto &frame_affine_brightness = frame.affineBrightness();

  const double mean_square_optical_flow = frame.meanSquareOpticalFlow();
  const double mean_square_optical_flow_without_rotation = frame.meanSquareOpticalFlowWithoutRotation();

  if (rmse_ < 0) {
    rmse_ = frame.poseRmse();
  }

  bool need_new_keyframe =
      factor_ * (kMaxShiftWeight_ * mean_square_optical_flow +
                 kMaxShiftWithoutRotationWeight_ * mean_square_optical_flow_without_rotation +
                 kMaxAffineBrightnessWeight_ * std::abs(frame_affine_brightness[0] - last_kf_affine_brightness[0])) >
          kThreshold_ ||
      (frame.poseRmse() / rmse_ > kMaxExcessOffEnergy_);

  need_new_keyframe &= frame.reliable();

  if (need_new_keyframe) {
    rmse_ = -1;
  }

  return need_new_keyframe;
}

template <energy::motion::Motion Motion>
MeanSquareOpticalFlowAndRmseKeyframeStrategy<Motion>::~MeanSquareOpticalFlowAndRmseKeyframeStrategy() = default;

template class MeanSquareOpticalFlowAndRmseKeyframeStrategy<energy::motion::SE3<Precision>>;

}  // namespace keyframe_strategy
}  // namespace tracker
}  // namespace dsopp
