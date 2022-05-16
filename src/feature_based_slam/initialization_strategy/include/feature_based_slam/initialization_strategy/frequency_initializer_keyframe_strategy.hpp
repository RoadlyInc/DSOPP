#ifndef DSOPPP_FEATURE_BASED_SLAM_FREQUENCY_KEYFRAME_STRATEGY_HPP_
#define DSOPPP_FEATURE_BASED_SLAM_FREQUENCY_KEYFRAME_STRATEGY_HPP_

#include "common/settings.hpp"
#include "feature_based_slam/initialization_strategy/initializer_keyframe_strategy.hpp"

namespace dsopp::feature_based_slam::initialization_strategy {

/**
 * \brief Keyframe strategy that stops getting new frames after `period_ms_` milliseconds
 */
class FrequencyInitializerKeyframeStrategy : public InitializerKeyframeStrategy {
 public:
  /**
   * creates FrequencyInitializerKeyframeStrategt with a given frequency
   * @param frequency number of keyframes per second to be created
   */
  explicit FrequencyInitializerKeyframeStrategy(Precision frequency);

  FrameStatus needNewFrame(const dsopp::features::CameraFeatures &new_frame,
                           const std::deque<track::Frame> &previous_frames, const size_t rotation_ransac_inliers,
                           const size_t number_of_matches) override;

  bool managesStandstill() override;

  ~FrequencyInitializerKeyframeStrategy() override;

 private:
  /** time between first and last frames in milliseconds - number inverse to frequency */
  const long period_ms_;
};

}  // namespace dsopp::feature_based_slam::initialization_strategy

#endif  // DSOPPP_FEATURE_BASED_SLAM_FREQUENCY_KEYFRAME_STRATEGY_HPP_
