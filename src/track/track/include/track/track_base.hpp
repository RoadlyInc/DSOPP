#ifndef DSOPP_TRACK_BASE_HPP
#define DSOPP_TRACK_BASE_HPP

#include <map>
#include <memory>

#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"

namespace dsopp {
namespace sanity_checker {
enum struct SanityCheckStatus;
}  // namespace sanity_checker
namespace sensors {
class AgentSettings;
}
namespace track {
class GnssTrack;
class ECEFPoses;

namespace storage {
class Track;
}

/**
 * \brief Track interface.
 *
 * Class to represent track. All types of the tracks should implement this interface.
 * @tparam OdometryTrackType type of odometry track
 * @tparam Motion Motion type
 */
template <template <class> class OdometryTrackType, energy::motion::Motion Motion>
class TrackBase {
 public:
  /** exported motion type */
  using MotionT = Motion;

  TrackBase();

  /**
   * creates track storage which contains protobuf containers
   * @return track storage
   */
  virtual storage::Track serialize() const = 0;

  /**
   * get odometry track
   * @return odometry track
   */
  OdometryTrackType<Motion> &odometryTrack() const;

  /**
   * get agent settings
   * @return agent settings
   */
  sensors::AgentSettings &agentSettings() const;
  /**
   * get sanity check results
   * @return sanity check results
   */
  const std::map<size_t, sanity_checker::SanityCheckStatus> &sanityCheckResults() const;

  /**
   * set sanity check results
   * @param sanity_check_results sanity check results
   */
  void setSanityCheckResults(std::map<size_t, sanity_checker::SanityCheckStatus> &&sanity_check_results);

  virtual ~TrackBase();

 protected:
  /** agent settings */
  std::unique_ptr<sensors::AgentSettings> agent_settings_;
  /** odometry track*/
  std::unique_ptr<OdometryTrackType<Motion>> odometry_track_;
  /** sanity check results*/
  std::map<size_t, sanity_checker::SanityCheckStatus> sanity_check_results_;
};

}  // namespace track
}  // namespace dsopp

#endif  // DSOPP_TRACK_BASE_HPP
