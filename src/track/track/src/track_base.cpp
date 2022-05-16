#include "track/track_base.hpp"

#include "agent/agent_settings.hpp"

#include "sanity_checker/sanity_checker.hpp"
#include "track/active_odometry_track.hpp"
#include "track/odometry_track.hpp"

namespace dsopp {
namespace track {
template <template <class> class OdometryTrackType, energy::motion::Motion Motion>
TrackBase<OdometryTrackType, Motion>::TrackBase() = default;

template <template <class> class OdometryTrackType, energy::motion::Motion Motion>
OdometryTrackType<Motion> &TrackBase<OdometryTrackType, Motion>::odometryTrack() const {
  return *odometry_track_;
}

template <template <class> class OdometryTrackType, energy::motion::Motion Motion>
const std::map<size_t, sanity_checker::SanityCheckStatus> &TrackBase<OdometryTrackType, Motion>::sanityCheckResults()
    const {
  return sanity_check_results_;
}

template <template <class> class OdometryTrackType, energy::motion::Motion Motion>
sensors::AgentSettings &TrackBase<OdometryTrackType, Motion>::agentSettings() const {
  return *agent_settings_;
}

template <template <class> class OdometryTrackType, energy::motion::Motion Motion>
void TrackBase<OdometryTrackType, Motion>::setSanityCheckResults(
    std::map<size_t, sanity_checker::SanityCheckStatus> &&sanity_check_results) {
  sanity_check_results_ = std::move(sanity_check_results);
}

template <template <class> class OdometryTrackType, energy::motion::Motion Motion>
TrackBase<OdometryTrackType, Motion>::~TrackBase() = default;

template class TrackBase<ActiveOdometryTrack, energy::motion::SE3<Precision>>;
template class TrackBase<OdometryTrack, energy::motion::SE3<Precision>>;

}  // namespace track
}  // namespace dsopp
