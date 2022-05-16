#include "track/track.hpp"

#include "agent/agent_settings.hpp"

#include "storage/track_storage.hpp"
#include "track/odometry_track.hpp"

namespace dsopp {
namespace track {
template <energy::motion::Motion Motion>
Track<Motion>::Track(std::unique_ptr<sensors::AgentSettings> &&agent_settings,
                     std::unique_ptr<OdometryTrack<Motion>> &&odometry_track,
                     std::map<size_t, sanity_checker::SanityCheckStatus> &&sanity_check_results) {
  this->agent_settings_ = std::move(agent_settings);
  this->odometry_track_ = std::move(odometry_track);
  this->sanity_check_results_ = std::move(sanity_check_results);
}

template <energy::motion::Motion Motion>
Track<Motion>::Track(const storage::Track &track_storage)
    : Track(std::make_unique<sensors::AgentSettings>(track_storage.agentSettings()),
            std::make_unique<OdometryTrack<Motion>>(track_storage.odometryTrack())) {
  for (const auto &[frame_index, result] : track_storage.sanityCheckResults().sanity_check_results()) {
    this->sanity_check_results_[frame_index] = sanity_checker::SanityCheckStatus(result);
  }
}

template <energy::motion::Motion Motion>
storage::Track Track<Motion>::serialize() const {
  return storage::Track(proto::GnssTrack(), proto::ECEFPoses(), this->sanity_check_results_,
                        this->agent_settings_->proto(), this->odometry_track_->serialize());
}

template <energy::motion::Motion Motion>
Track<Motion>::~Track() = default;

template class Track<energy::motion::SE3<Precision>>;

}  // namespace track
}  // namespace dsopp
