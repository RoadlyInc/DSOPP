#include "feature_based_slam/initialization_strategy/wait_for_movement_keyframe_strategy.hpp"

#include <glog/logging.h>

namespace dsopp::feature_based_slam::initialization_strategy {

WaitForMovementInitializerKeyframeStrategy::WaitForMovementInitializerKeyframeStrategy() {}

WaitForMovementInitializerKeyframeStrategy::WaitForMovementInitializerKeyframeStrategy(size_t sliding_window_length,
                                                                                       Precision rotation_inlier_ratio)
    : sliding_window_length_(sliding_window_length), rotation_inlier_ratio_(rotation_inlier_ratio) {}

FrameStatus WaitForMovementInitializerKeyframeStrategy::needNewFrame(const dsopp::features::CameraFeatures &new_frame,
                                                                     const std::deque<track::Frame> &previous_frames,
                                                                     const size_t rotation_ransac_inliers,
                                                                     const size_t number_of_matches) {
  if (previous_frames.size() == 0) return FrameStatus::kKeepFrame;

  VLOG(1) << "Rotation inliers : " << rotation_ransac_inliers << " | " << number_of_matches;
  if (rotation_ransac_inliers >
      static_cast<size_t>(rotation_inlier_ratio_ * static_cast<Precision>(number_of_matches))) {
    VLOG(1) << "Skipping frame " << new_frame.id();
    return FrameStatus::kDropFrame;
  }
  if (sliding_window_length_ <= previous_frames.size() + 1) {
    return FrameStatus::kFinish;
  }
  return FrameStatus::kKeepFrame;
}

bool WaitForMovementInitializerKeyframeStrategy::managesStandstill() { return true; }

WaitForMovementInitializerKeyframeStrategy::~WaitForMovementInitializerKeyframeStrategy() = default;
}  // namespace dsopp::feature_based_slam::initialization_strategy
