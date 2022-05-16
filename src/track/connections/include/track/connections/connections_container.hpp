
#ifndef DSOPP_CONNECTIONS_CONTAINER_HPP
#define DSOPP_CONNECTIONS_CONTAINER_HPP

#include "connection.pb.h"
#include "energy/motion/motion.hpp"

#include <map>
#include <memory>
#include <vector>

namespace dsopp {
namespace track {
template <energy::motion::MotionProduct MotionProduct>
class FrameConnection;
/** class storing and managing consistency of frame connections
 *
 * @tparam Motion Motion type
 */
template <energy::motion::MotionProduct MotionProduct>
class ConnectionsContainer {
 public:
  /** structure for accessing and lookup of frame connections*/
  using Connections = std::map<size_t, std::map<size_t, std::unique_ptr<FrameConnection<MotionProduct>>>>;
  /** structure for accessing and lookup of frame connections*/
  using ConnectionsLookup = std::map<size_t, std::map<size_t, FrameConnection<MotionProduct>*>>;

  ConnectionsContainer();
  /**
   * creates a frame connection from proto container
   * @param proto proto container
   */
  explicit ConnectionsContainer(const proto::Connections& proto);
  /**
   * copy constructor
   * @param connections connections container
   */
  explicit ConnectionsContainer(const ConnectionsContainer& connections);
  ~ConnectionsContainer();
  /**
   * get a frame connection of a queried pair, assumed to be used only on a valid pair
   * @param reference_keyframe_id,target_keyframe_id id of the queried frames
   * @return queried connection
   * @throws out of bound exception when an invalid connection is queried
   */
  FrameConnection<MotionProduct>& get(size_t reference_keyframe_id, size_t target_keyframe_id) const;

  /**
   * @param keyframe_id quiered keyframe id
   * @return all connections of a given frame
   */
  std::vector<const FrameConnection<MotionProduct>*> get(size_t keyframe_id) const;
  /**
   * adds connection
   * @param connection connection to add
   */
  void add(std::unique_ptr<FrameConnection<MotionProduct>>&& connection);
  /**
   * @return true if connections between frames exists and false otherwise
   * @param reference_keyframe_id, target_keyframe_id frame to check
   */
  bool exists(size_t reference_keyframe_id, size_t target_keyframe_id) const;
  /**
   * creates protobuf container from the object
   * @return protobuf container
   */
  proto::Connections proto() const;

 private:
  /** connections of frames (only for keyframes)*/
  Connections connections_;
  /** lookup connections of frames for accessing connections in a reverse order*/
  ConnectionsLookup connections_lookup_;
};
}  // namespace track
}  // namespace dsopp
#endif  // DSOPP_CONNECTIONS_CONTAINER_HPP
