#include "storage/track_storage.hpp"

#include <fstream>
#include <vector>

#include "agent/agent_settings.hpp"
#include "common/settings.hpp"

namespace dsopp::track::storage {

namespace {

template <class ProtoMessage>
void saveProto(std::ostream &stream, const ProtoMessage &proto_message) {
#if GOOGLE_PROTOBUF_VERSION >= 3010000
  unsigned message_size = static_cast<unsigned>(proto_message.ByteSizeLong());
#else
  unsigned message_size = static_cast<unsigned>(proto_message.ByteSize());
#endif
  stream.write(reinterpret_cast<char *>(&message_size), sizeof(unsigned));
  proto_message.SerializeToOstream(&stream);
}

template <class ProtoMessage>
bool readProto(std::istream &stream, ProtoMessage &proto_message) {
  unsigned message_size;
  stream.read(reinterpret_cast<char *>(&message_size), sizeof(unsigned));
  std::vector<char> message_buffer(message_size);
  stream.read(message_buffer.data(), message_size);
  if (!proto_message.ParseFromArray(message_buffer.data(), static_cast<int>(message_size))) return false;
  return true;
}

}  // namespace

storage::Track::Track() = default;

Track::Track(const proto::GnssTrack &gnss_track, const proto::ECEFPoses &ecef_poses,
             const std::map<size_t, sanity_checker::SanityCheckStatus> &sanity_check_results,
             const dsopp::sensors::proto::AgentSettings &agent_settings, const OdometryTrack &odometry_track) {
  this->gnss_track_ = gnss_track;
  this->ecef_poses_ = ecef_poses;
  for (const auto &[frame_index, result] : sanity_check_results) {
    (*this->sanity_check_results_.mutable_sanity_check_results())[frame_index] = proto::SanityCheckStatus(result);
  }
  this->agent_settings_ = agent_settings;
  this->odometry_track_ = odometry_track;
}

void storage::Track::save(std::ostream &stream) const {
  long unsigned number_of_frames = this->odometry_track_.frames.size();
  stream.write(reinterpret_cast<char *>(&number_of_frames), sizeof(long unsigned));
  for (auto &frame : this->odometry_track_.frames) saveProto(stream, frame);

  saveProto(stream, this->odometry_track_.connections);
  saveProto(stream, this->gnss_track_);
  saveProto(stream, this->ecef_poses_);
  saveProto(stream, this->sanity_check_results_);
  saveProto(stream, this->agent_settings_);
}

bool storage::Track::read(std::istream &stream) {
  long unsigned number_of_frames;
  stream.read(reinterpret_cast<char *>(&number_of_frames), sizeof(long unsigned));
  this->odometry_track_.frames.resize(number_of_frames);
  for (size_t i = 0; i < number_of_frames; ++i) {
    if (!readProto(stream, this->odometry_track_.frames[i])) return false;
  }

  if (!readProto(stream, this->odometry_track_.connections)) return false;
  if (!readProto(stream, this->gnss_track_)) return false;
  if (!readProto(stream, this->ecef_poses_)) return false;
  if (!readProto(stream, this->sanity_check_results_)) return false;
  if (!readProto(stream, this->agent_settings_)) return false;
  return true;
}

const proto::GnssTrack &storage::Track::gnssTrack() const { return this->gnss_track_; }

const proto::ECEFPoses &storage::Track::ecefPoses() const { return this->ecef_poses_; }

const proto::SanityCheckResults &storage::Track::sanityCheckResults() const { return this->sanity_check_results_; }

const dsopp::sensors::proto::AgentSettings &storage::Track::agentSettings() const { return this->agent_settings_; }

const OdometryTrack &storage::Track::odometryTrack() const { return this->odometry_track_; }
}  // namespace dsopp::track::storage
