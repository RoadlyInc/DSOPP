#ifndef DSOPPP_FEATURE_BASED_SLAM_INITIALIZATION_STRATEGY_HPP_
#define DSOPPP_FEATURE_BASED_SLAM_INITIALIZATION_STRATEGY_HPP_

#include "feature_based_slam/track/frame.hpp"
#include "features/camera/camera_features.hpp"

#include <vector>

namespace dsopp::feature_based_slam::initialization_strategy {
/**
 * status of frame
 * kDropFrame -- drop current frame from initialization optimization routine
 * kKeepFrame -- keep current take next
 * kFinish -- this frame was last
 */
enum class FrameStatus : unsigned int { kDropFrame, kKeepFrame, kFinish };

/**
 * \brief Base class for initialization keyframe strategy
 *
 */
class InitializerKeyframeStrategy {
 public:
  /**
   * method to check if we neeed one more frame to start initialization
   * @param new_frame new frame data
   * @param previous_frames previous frames data
   * @param rotation_ransac_inliers inliers from SO3 only ransac
   * @param number_of_matches optical number of matches with previous frame
   * @return frame status from FrameStatus enum
   */
  virtual FrameStatus needNewFrame(const dsopp::features::CameraFeatures &new_frame,
                                   const std::deque<track::Frame> &previous_frames,
                                   const size_t rotation_ransac_inliers, const size_t number_of_matches) = 0;

  /**
   * method which indicates if strategy needs rotation ransac inliers
   *
   * @return true if strategy needs SO3-ransac
   */
  virtual bool managesStandstill() = 0;

  virtual ~InitializerKeyframeStrategy() = default;
};

}  // namespace dsopp::feature_based_slam::initialization_strategy

#endif  // DSOPPP_FEATURE_BASED_SLAM_INITIALIZATION_STRATEGY_HPP_
