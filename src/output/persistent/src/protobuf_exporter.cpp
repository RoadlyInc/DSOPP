#include "output/persistent/protobuf_exporter.hpp"

#include <fstream>

#include "common/settings.hpp"

#include "track/active_odometry_track.hpp"
#include "track/active_track.hpp"
#include "track/odometry_track.hpp"
#include "track/track.hpp"

namespace dsopp {
namespace output {
namespace {
void save(const track::storage::Track &track_storage, const std::string &file) {
  std::ofstream stream(file, std::ios::binary);
  if (stream.is_open()) track_storage.save(stream);
}
}  // namespace

template <template <class> class OdometryTrackType, energy::motion::Motion Motion>
ProtobufExporter<OdometryTrackType, Motion>::ProtobufExporter(const std::string &file, size_t save_stride)
    : file_(file), save_stride_(save_stride), current_keyframes_(0) {}

template <template <class> class OdometryTrackType, energy::motion::Motion Motion>
void ProtobufExporter<OdometryTrackType, Motion>::notify(const track::TrackBase<OdometryTrackType, Motion> &track) {
  if (save_stride_ == 0) return;

  const size_t keyframes = track.odometryTrack().keyframes().size();
  if (keyframes != current_keyframes_) {
    if (keyframes % save_stride_ == 0) {
      track_storage_ = track.serialize();
      save(track_storage_, file_);
      current_keyframes_ = keyframes;
    }
  }
}
template <template <class> class OdometryTrackType, energy::motion::Motion Motion>
void ProtobufExporter<OdometryTrackType, Motion>::finish(const track::TrackBase<OdometryTrackType, Motion> &track) {
  track_storage_ = track.serialize();
  save(track_storage_, file_);
}

template <template <class> class OdometryTrackType, energy::motion::Motion Motion>
ProtobufExporter<OdometryTrackType, Motion>::~ProtobufExporter() = default;

template class ProtobufExporter<track::OdometryTrack, energy::motion::SE3<Precision>>;
template class ProtobufExporter<track::ActiveOdometryTrack, energy::motion::SE3<Precision>>;

}  // namespace output
}  // namespace dsopp
