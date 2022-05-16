#ifndef DSOPPP_FEATURE_BASED_SLAM_WAIT_FOR_MOVEMENT_INITIALIZATION_STRATEGY_HPP_
#define DSOPPP_FEATURE_BASED_SLAM_WAIT_FOR_MOVEMENT_INITIALIZATION_STRATEGY_HPP_

#include "common/settings.hpp"
#include "feature_based_slam/initialization_strategy/initializer_keyframe_strategy.hpp"

namespace dsopp::feature_based_slam::initialization_strategy {

/**
 * \brief Keyframe strategy which skips frames for which translation is significantly small
 * in all will push to initialization `sliding_window_length` frames
 *
 */
class WaitForMovementInitializerKeyframeStrategy : public InitializerKeyframeStrategy {
 public:
  /**
   * creates strategy with default paremeters
   */
  WaitForMovementInitializerKeyframeStrategy();
  /**
   * creates Keyframe strategy
   *
   * @param sliding_window_length length of sliding window to optimize in
   * @param rotation_inlier_ratio inliers ratio threshold for SO3 ransac
   */
  WaitForMovementInitializerKeyframeStrategy(size_t sliding_window_length, Precision rotation_inlier_ratio);

  FrameStatus needNewFrame(const dsopp::features::CameraFeatures &new_frame,
                           const std::deque<track::Frame> &previous_frames, const size_t rotation_ransac_inliers,
                           const size_t number_of_matches) override;

  bool managesStandstill() override;

  ~WaitForMovementInitializerKeyframeStrategy() override;

 private:
  /** Number of frames to start estimation */
  const size_t sliding_window_length_ = 5;
  /** inliers ratio to reject non zero translation hypotesis */
  const Precision rotation_inlier_ratio_ = 0.7_p;
};

}  // namespace dsopp::feature_based_slam::initialization_strategy

#endif  // DSOPPP_FEATURE_BASED_SLAM_WAIT_FOR_MOVEMENT_INITIALIZATION_STRATEGY_HPP_
