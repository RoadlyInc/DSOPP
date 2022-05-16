
#ifndef DSOPP_ODOMETRY_TRACK_HPP
#define DSOPP_ODOMETRY_TRACK_HPP

#include <memory>

#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"
#include "odometry_track.pb.h"
#include "storage/track_storage.hpp"
#include "track/frames/keyframe.hpp"
#include "track/odometry_track_base.hpp"

namespace dsopp {
namespace track {
template <energy::motion::Motion Motion>
class Keyframe;
/**
 * OdometryTrack that can not longer be modified and doesn't store additional data for optimization.
 * @tparam Motion Motion type
 */
template <energy::motion::Motion Motion>
class OdometryTrack : public OdometryTrackBase<Motion> {
 public:
  /**
   * creates result odometry track from the set of frames
   * @param frames marginalized frames
   * @param connections connections between frames
   */
  OdometryTrack(std::vector<std::unique_ptr<Keyframe<Motion>>>&& frames,
                std::unique_ptr<ConnectionsContainer<typename Motion::Product>>&& connections);
  /**
   * creates result odometry track from the protobuf containers
   * @param odometry_track container for the odometry track
   */
  explicit OdometryTrack(const storage::OdometryTrack& odometry_track);
  /**
   * get all frames from the odometry track.
   * @return all frames
   */
  const std::vector<const Keyframe<Motion>*> keyframes() const;
  /**
   * creates protobuf containers from the object
   * @return protobuf containers
   */
  storage::OdometryTrack serialize() const;

 private:
  /** marginalized states of the agent at different time*/
  std::vector<std::unique_ptr<Keyframe<Motion>>> frames_;
};

}  // namespace track
}  // namespace dsopp
#endif  // DSOPP_ODOMETRY_TRACK_HPP
