#ifndef DSOPP_SLAM_INTERNAL_TRACKING_FRAME_HPP
#define DSOPP_SLAM_INTERNAL_TRACKING_FRAME_HPP

#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"
#include "track/frames/tracking_frame.hpp"

#include "common/settings.hpp"

namespace dsopp {
namespace track {
/**
 * This type of tracking frame is created using the slam algorithm.
 * @tparam Motion Motion type
 */
template <energy::motion::Motion Motion>
class SLAMInternalTrackingFrame : public TrackingFrame<Motion> {
 public:
  /**
   * @param id frame id
   * @param timestamp timestamps of the frame
   * @param t_world_keyframe transformation from keyframe to world
   * @param t_keyframe_agent transformation from agent to keyframe
   * @param affine_brightness affine brightness
   * @param mean_square_optical_flow mean square optical flow from keyframe to frame
   * @param mean_square_optical_flow_without_rotation mean square optical flow without rotation from keyframe to frame
   * @param pose_rmse mean frame energy relative to a keyframe
   * @param reliable pose estimation was reliable
   */
  SLAMInternalTrackingFrame(size_t id, time timestamp, const Motion& t_world_keyframe,
                            const typename Motion::Product& t_keyframe_agent,
                            const Eigen::Vector<Precision, 2>& affine_brightness,
                            Precision mean_square_optical_flow = 0,
                            Precision mean_square_optical_flow_without_rotation = 0, Precision pose_rmse = 0,
                            bool reliable = true);

  /**
   * @return mean square optical flow
   */
  Precision meanSquareOpticalFlow() const;

  /**
   * @return mean square optical flow without rotation
   */
  Precision meanSquareOpticalFlowWithoutRotation() const;

  /**
   * @return mean frame energy relative to a keyframe
   */
  Precision poseRmse() const;

  /**
   * @return pose estimation was reliable
   */
  bool reliable() const;

 private:
  /** mean square optical flow from keyframe to frame */
  Precision mean_square_optical_flow_;
  /** mean square optical flow without rotation from keyframe to frame */
  Precision mean_square_optical_flow_without_rotation_;
  /** mean frame energy relative to a keyframe */
  Precision pose_rmse_;
  /** pose estimation was reliable */
  bool reliable_;
};

}  // namespace track
}  // namespace dsopp

#endif  // DSOPP_SLAM_INTERNAL_TRACKING_FRAME_HPP
