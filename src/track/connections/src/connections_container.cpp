#include "track/connections/connections_container.hpp"

#include "energy/motion/se3_motion.hpp"

#include "track/connections/frame_connection.hpp"

namespace dsopp {
namespace track {
template <energy::motion::MotionProduct MotionProduct>
FrameConnection<MotionProduct> &ConnectionsContainer<MotionProduct>::get(size_t reference_frame_id,
                                                                         size_t target_frame_id) const {
  if (reference_frame_id > target_frame_id) {
    std::swap(reference_frame_id, target_frame_id);
  }
  return *connections_.at(reference_frame_id).at(target_frame_id);
}

template <energy::motion::MotionProduct MotionProduct>
ConnectionsContainer<MotionProduct>::ConnectionsContainer(const proto::Connections &proto) {
  for (int i = 0; i < proto.connections_size(); i++) {
    add(std::make_unique<FrameConnection<MotionProduct>>(proto.connections(i)));
  }
}

template <energy::motion::MotionProduct MotionProduct>
ConnectionsContainer<MotionProduct>::ConnectionsContainer(const ConnectionsContainer &connections) {
  for (auto &[reference_id, reference_connection] : connections.connections_) {
    for (auto &[target_id, connection] : reference_connection) {
      connections_[reference_id][target_id] = std::make_unique<FrameConnection<MotionProduct>>(*connection);
      connections_lookup_[target_id][reference_id] = connections_[reference_id][target_id].get();
    }
  }
}

template <energy::motion::MotionProduct MotionProduct>
void ConnectionsContainer<MotionProduct>::add(std::unique_ptr<FrameConnection<MotionProduct>> &&connection) {
  connections_lookup_[connection->targetKeyframeId()][connection->referenceKeyframeId()] = connection.get();
  connections_[connection->referenceKeyframeId()][connection->targetKeyframeId()] = std::move(connection);
}

template <energy::motion::MotionProduct MotionProduct>
bool ConnectionsContainer<MotionProduct>::exists(size_t reference_frame_id, size_t target_frame_id) const {
  if (reference_frame_id > target_frame_id) {
    std::swap(reference_frame_id, target_frame_id);
  }
  return ((connections_.count(reference_frame_id) > 0) and
          (connections_.at(reference_frame_id).count(target_frame_id) > 0));
}

template <energy::motion::MotionProduct MotionProduct>
std::vector<const FrameConnection<MotionProduct> *> ConnectionsContainer<MotionProduct>::get(size_t frame_id) const {
  std::vector<const FrameConnection<MotionProduct> *> connections;
  if (connections_.find(frame_id) != connections_.end()) {
    for (const auto &[id, connection] : connections_.at(frame_id)) {
      connections.push_back(connection.get());
    }
  }
  if (connections_lookup_.find(frame_id) != connections_lookup_.end()) {
    for (const auto &[id, connection] : connections_lookup_.at(frame_id)) {
      connections.push_back(connection);
    }
  }
  return connections;
}

template <energy::motion::MotionProduct MotionProduct>
proto::Connections ConnectionsContainer<MotionProduct>::proto() const {
  proto::Connections connections;
  for (const auto &[key1, connections_reference] : connections_) {
    for (const auto &[key2, connection] : connections_reference) {
      *connections.add_connections() = connection->proto();
    }
  }
  return connections;
}

template <energy::motion::MotionProduct MotionProduct>
ConnectionsContainer<MotionProduct>::ConnectionsContainer() = default;

template <energy::motion::MotionProduct MotionProduct>
ConnectionsContainer<MotionProduct>::~ConnectionsContainer() = default;

template class ConnectionsContainer<energy::motion::SE3<Precision>>;

}  // namespace track
}  // namespace dsopp
