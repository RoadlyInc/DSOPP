
#ifndef DSOPP_ACTIVE_ODOMETRY_TRACK_HPP
#define DSOPP_ACTIVE_ODOMETRY_TRACK_HPP

#include <deque>
#include <memory>
#include <vector>

#include "common/settings.hpp"
#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"
#include "odometry_track.pb.h"
#include "track/frames/active_keyframe.hpp"
#include "track/odometry_track_base.hpp"

namespace dsopp {
namespace track {
template <energy::motion::Motion Motion>
class OdometryTrack;
/**
 * \brief Active odometry track.
 *
 * This type of odometry track is created using the slam algorithm.
 * @tparam Motion Motion type
 */
template <energy::motion::Motion Motion>
class ActiveOdometryTrack : public OdometryTrackBase<Motion> {
 public:
  /** shortcut for the immature landmark activation statuses per frame */
  using ImmatureLandmarkActivationStatusesPerSensor =
      std::map<size_t, std::vector<typename ActiveKeyframe<Motion>::ImmatureLandmarkActivationStatus>>;
  ActiveOdometryTrack();
  /**
   * adds new immature frame to the track
   * @param id frame id
   * @param timestamp time of the capture
   * @param t_world_agent pose of the agent
   * @param affine_brightness affine brightness
   */
  void pushFrame(size_t id, time timestamp, const Motion& t_world_agent,
                 const Eigen::Vector2<Precision>& affine_brightness = Eigen::Vector2<Precision>::Zero());
  /**
   * get all keyframes
   *
   * @return all keyframes
   */
  const std::vector<ActiveKeyframe<Motion>*> keyframes() const;
  /**
   * get all marginalized frames with const reference
   *
   * @return all marginalized frames
   */
  const std::deque<ActiveKeyframe<Motion>*>& marginalizedFrames() const;

  /**
   * @param index index of marginalized frame
   *
   * @return marginalized frames
   */
  const ActiveKeyframe<Motion>& marginalizedFrame(size_t index) const;
  /**
   * get all active frames with const reference
   *
   * @return all active frames
   */
  const std::deque<ActiveKeyframe<Motion>*>& activeFrames() const;
  /**
   * @return last keyframe in the track
   */
  ActiveKeyframe<Motion>& lastKeyframe() const;
  /**
   * @param frame_id active frame id to return
   * @return active odometry track with the given id
   */
  ActiveKeyframe<Motion>& getActiveKeyframe(const size_t frame_id) const;
  /**
   * @return true if there is no frames in the track
   */
  bool empty() const;
  /**
   * marginalizes frames with given frame id
   * @param frame_id frame to be marginalized
   */
  void marginalizeFrame(const size_t frame_id);
  /**
   * clears recources for all marginalized frames
   */
  void unloadMarginalizedResources();
  /**
   * apply activation statuses to immature landmarks on each frame
   * @param statuses new statuses
   */
  void applyImmatureLandmarkActivationStatuses(std::map<size_t, ImmatureLandmarkActivationStatusesPerSensor>& statuses);
  /**
   * Return the n-th frame in the track. If n is below zero, it will be an n-th frame from the end (n = -1 is the last
   * frame).
   *
   * @param n number of the frame
   * @return n-th frame. If something wrong (empty track, etc.) return nullptr.
   */
  const Frame<Motion>* getFrame(int n) const;
  /**
   * @return frames
   */
  const std::vector<const ActiveKeyframe<Motion>*> frames() const;
  ~ActiveOdometryTrack();

  // TODO: think about how to rename it
  /**
   * method to create `OdometryTrack` class instance
   * @param legends semantics legends to get tag for the landmarks. Each legend can be nullptr.
   * @return unique ptr to `OdometryTrack`
   */
  std::unique_ptr<OdometryTrack<Motion>> createTrack(
      const std::map<size_t, const semantics::SemanticLegend*>& legends) const;

 private:
  /** keyframes of the agent at different time*/
  std::vector<std::unique_ptr<ActiveKeyframe<Motion>>> frames_;
  /** active keyframes of the agent at different time*/
  std::deque<ActiveKeyframe<Motion>*> active_frames_;
  /** marginalized keyframes of the agent at different time*/
  std::deque<ActiveKeyframe<Motion>*> marginalized_frames_;
};

}  // namespace track
}  // namespace dsopp

#endif  // DSOPP_ACTIVE_ODOMETRY_TRACK_HPP
