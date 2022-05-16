#ifndef DSOPP_SRC_TRACK_ODOMETRY_TRACK_BASE_HPP_
#define DSOPP_SRC_TRACK_ODOMETRY_TRACK_BASE_HPP_

#include "energy/motion/motion.hpp"

#include <memory>

namespace dsopp {
namespace track {
template <energy::motion::MotionProduct MotionProduct>
class ConnectionsContainer;
/**
 * \brief OdometryTrack interface.
 *
 * Class to represent odometry track of the agent (movement of the camera in the time). All types of the odometry tracks
 * should implement this interface.
 *
 * @tparam Motion Motion type
 */
template <energy::motion::Motion Motion>
class OdometryTrackBase {
 public:
  OdometryTrackBase();
  /**
   * creates track base with the given connections
   * @param connections connections between frames
   */
  OdometryTrackBase(std::unique_ptr<ConnectionsContainer<typename Motion::Product>>&& connections);
  ~OdometryTrackBase();
  /**
   * @return connections of the odometry track
   */
  ConnectionsContainer<typename Motion::Product>& connections();

 protected:
  /** connections between different poses, provide additional information on relative poses*/
  std::unique_ptr<ConnectionsContainer<typename Motion::Product>> connections_;
};
}  // namespace track
}  // namespace dsopp

#endif  // DSOPP_SRC_TRACK_ODOMETRY_TRACK_BASE_HPP_
