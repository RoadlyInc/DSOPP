#ifndef DSOPP_LOCAL_ODOMETRY_TRACK_HPP
#define DSOPP_LOCAL_ODOMETRY_TRACK_HPP

#include <list>

#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"
#include "visualizer/local_odometry_frame.hpp"

namespace dsopp {
namespace track {
template <energy::motion::Motion Motion>
class OdometryTrack;
template <energy::motion::Motion Motion>
class ActiveOdometryTrack;
}  // namespace track

namespace output {
/**
 * Struct to store the local copy of the odometry track.
 */
struct LocalOdometryTrack {
  /**
   * create a local copy of the odometry track
   * @param track odometry track to copy
   */
  template <energy::motion::Motion Motion>
  LocalOdometryTrack(const track::OdometryTrack<Motion> &track);
  /**
   * create a local copy of the active track
   * @param track active odometry track to copy
   */
  template <energy::motion::Motion Motion>
  LocalOdometryTrack(const track::ActiveOdometryTrack<Motion> &track);
  LocalOdometryTrack() = default;
  /** local copy of the keyframes */
  std::list<LocalOdometryFrame> frames;
};
}  // namespace output
}  // namespace dsopp

#endif  // DSOPP_LOCAL_ODOMETRY_TRACK_HPP
