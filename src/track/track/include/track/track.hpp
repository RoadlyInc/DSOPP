#ifndef DSOPP_TRACK_HPP
#define DSOPP_TRACK_HPP

#include <memory>

#include "track/track_base.hpp"

namespace dsopp {
namespace track {
class GnssTrack;
class ECEFPoses;

template <energy::motion::Motion Motion>
class OdometryTrack;

/**
 * Track that can not longer be modified and doesn't store additional data for optimization.
 * @tparam Motion Motion type
 */
template <energy::motion::Motion Motion>
class Track final : public TrackBase<OdometryTrack, Motion> {
 public:
  /**
   * creates result track from the result odometry track, gnss track, ECEF poses and sanity check results
   * @param agent_settings settings of all sensors in the agent
   * @param odometry_track result odometry track
   * @param sanity_check_results sanity check results
   */
  Track(std::unique_ptr<sensors::AgentSettings> &&agent_settings,
        std::unique_ptr<OdometryTrack<Motion>> &&odometry_track,
        std::map<size_t, sanity_checker::SanityCheckStatus> &&sanity_check_results = {});

  /**
   * creates result track from the protobuf container
   * @param track_storage storage which contains protobuf messages
   */

  explicit Track(const storage::Track &track_storage);

  storage::Track serialize() const override;

  ~Track();
};

}  // namespace track
}  // namespace dsopp

#endif  // DSOPP_TRACK_HPP
