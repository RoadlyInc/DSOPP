#include <memory>
#include <vector>

#include "track/connections/connections_container.hpp"
#include "track/connections/frame_connection.hpp"
#include "track/odometry_track.hpp"

namespace dsopp {
namespace track {
template <energy::motion::Motion Motion>
OdometryTrack<Motion>::OdometryTrack(std::vector<std::unique_ptr<Keyframe<Motion>>> &&frames,
                                     std::unique_ptr<ConnectionsContainer<typename Motion::Product>> &&connections)
    : OdometryTrackBase<Motion>(std::move(connections)), frames_(std::move(frames)) {}

template <energy::motion::Motion Motion>
const std::vector<const Keyframe<Motion> *> OdometryTrack<Motion>::keyframes() const {
  std::vector<const Keyframe<Motion> *> frames;
  for (const auto &frame : frames_) {
    frames.push_back(frame.get());
  }
  return frames;
}

template <energy::motion::Motion Motion>
OdometryTrack<Motion>::OdometryTrack(const storage::OdometryTrack &odometry_track) {
  for (unsigned i = 0; i < odometry_track.frames.size(); i++) {
    frames_.emplace_back(std::make_unique<Keyframe<Motion>>(odometry_track.frames[i]));
  }
  this->connections_ = std::make_unique<ConnectionsContainer<typename Motion::Product>>(odometry_track.connections);
}

template <energy::motion::Motion Motion>
storage::OdometryTrack OdometryTrack<Motion>::serialize() const {
  storage::OdometryTrack odometry_track_proto;
  for (const auto &frame : frames_) {
    odometry_track_proto.frames.push_back(frame->proto());
  }
  odometry_track_proto.connections = this->connections_->proto();
  return odometry_track_proto;
}

template class OdometryTrack<energy::motion::SE3<Precision>>;

}  // namespace track
}  // namespace dsopp
