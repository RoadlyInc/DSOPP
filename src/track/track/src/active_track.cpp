#include "track/active_track.hpp"

#include "agent/agent_settings.hpp"

#include "storage/track_storage.hpp"
#include "track/active_odometry_track.hpp"
#include "track/odometry_track.hpp"
#include "track/track.hpp"

namespace dsopp {
namespace track {

template <energy::motion::Motion Motion>
ActiveTrack<Motion>::ActiveTrack() {
  this->agent_settings_ = std::make_unique<sensors::AgentSettings>();
  this->odometry_track_ = std::make_unique<ActiveOdometryTrack<Motion>>();
}

template <energy::motion::Motion Motion>
std::unique_ptr<Track<Motion>> ActiveTrack<Motion>::createTrack() const {
  // collect all legends for the settings
  std::map<size_t, const semantics::SemanticLegend *> legends;
  for (const auto &[sensor, settings] : this->agent_settings_->cameraSettings()) {
    if (settings.semanticLegend()) legends.insert({sensor, settings.semanticLegend()});
  }

  return std::make_unique<Track<Motion>>(
      std::make_unique<sensors::AgentSettings>(std::move(this->agent_settings_->clone())),
      this->odometry_track_->createTrack(legends));
}

template <energy::motion::Motion Motion>
storage::Track ActiveTrack<Motion>::serialize() const {
  return createTrack()->serialize();
}

template <energy::motion::Motion Motion>
ActiveTrack<Motion>::~ActiveTrack() = default;

template class ActiveTrack<energy::motion::SE3<Precision>>;

}  // namespace track
}  // namespace dsopp
