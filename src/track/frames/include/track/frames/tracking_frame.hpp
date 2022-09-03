#ifndef DSOPP_TRACKING_FRAME_HPP
#define DSOPP_TRACKING_FRAME_HPP

#include "common/settings.hpp"
#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"
#include "frame.pb.h"
#include "track/frames/frame.hpp"

namespace dsopp {
namespace track {
/**
 * Frame which is connected with a keyframe by a relative transformation
 * @tparam Motion Motion type
 */
template <energy::motion::Motion Motion>
class TrackingFrame : public Frame<Motion> {
 public:
  /**
   * @param id frame id
   * @param timestamp timestamps of the frame
   * @param t_world_keyframe transformation from keyframe to world
   * @param t_keyframe_agent transformation from agent to keyframe
   * @param exposure_time exposure time
   * @param affine_brightness affine brightness
   */
  TrackingFrame(size_t id, time timestamp, const Motion& t_world_keyframe,
                const typename Motion::Product& t_keyframe_agent, const Precision exposure_time,
                const Eigen::Vector<Precision, 2>& affine_brightness);

  /**
   * creates protobuf container from the object
   * @return protobuf container
   */
  proto::TrackingFrame proto() const;
  /**
   * notification point for new keyframe transformation
   * @param t_world_keyframe new keyframe transformation
   */
  void onKeyframeTransformationChanged(const Motion& t_world_keyframe);
  /**
   * @return transformation from keyframe's coordinate system to world
   */
  const Motion& tWorldKeyframe() const;
  /**
   * @return transformation from agent's coordinate system to keyframe's
   */
  const typename Motion::Product& tKeyframeAgent() const;
  /**
   * @param[out] frame_points_storage all points in the frame for the visualization
   */
  void points(std::map<std::string, storage::PointsStorage>& frame_points_storage) const;

 protected:
  /** attached key frame to frame transformation */
  Motion tWorldKeyframe_;
  /** Agent to the attached key frame transformation */
  const typename Motion::Product tKeyframeAgent_;
};

}  // namespace track
}  // namespace dsopp

#endif  // DSOPP_TRACKING_FRAME_HPP
