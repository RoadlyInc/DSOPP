#ifndef DSOPP_TRACK_STORAGE_HPP
#define DSOPP_TRACK_STORAGE_HPP
#include "agent_settings.pb.h"
#include "ecef_poses.pb.h"
#include "gnss_track.pb.h"
#include "odometry_track.pb.h"
#include "output_interfaces/track_output_interface.hpp"
#include "sanity_check_results.pb.h"
#include "sanity_checker/sanity_check_status.hpp"

namespace dsopp::track::storage {
/**protobuf containers for the odometry track */
struct OdometryTrack {
  /** protobuf container for the frames*/
  std::vector<proto::Keyframe> frames;
  /** protobuf container for the connections*/
  proto::Connections connections;
};
/** class for storing results */
class Track {
 public:
  Track();

  /**
   * @param gnss_track protobuf container for the gnss track
   * @param ecef_poses protobuf container for the ecef poses
   * @param sanity_check_results protobuf container for the sanity check results
   * @param agent_settings protobuf container for the agent settings
   * @param odometry_track container for the odometry track
   */
  Track(const proto::GnssTrack &gnss_track, const proto::ECEFPoses &ecef_poses,
        const std::map<size_t, sanity_checker::SanityCheckStatus> &sanity_check_results,
        const dsopp::sensors::proto::AgentSettings &agent_settings, const OdometryTrack &odometry_track);

  /**
   * saves results in stream
   * @param stream output stream
   */
  void save(std::ostream &stream) const;

  /**
   * reads data from stream
   * @param stream input stream
   * @return true on success
   */
  bool read(std::istream &stream);

  /**
   * get gnss track in proto format
   * @return gnss track
   */
  const proto::GnssTrack &gnssTrack() const;

  /**
   * get ecef poses in proto format
   * @return ecef poses
   */
  const proto::ECEFPoses &ecefPoses() const;

  /**
   * get sanity check results in proto format
   * @return sanity check results
   */
  const proto::SanityCheckResults &sanityCheckResults() const;

  /**
   * get agent settings in proto format
   * @return agent settings
   */
  const dsopp::sensors::proto::AgentSettings &agentSettings() const;

  /**
   * get odometry track container
   * @return odometry track
   */
  const OdometryTrack &odometryTrack() const;

 private:
  /** protobuf container for the gnss track */
  proto::GnssTrack gnss_track_;
  /** protobuf container for the ecef poses */
  proto::ECEFPoses ecef_poses_;
  /** protobuf container for the sanity check results */
  proto::SanityCheckResults sanity_check_results_;
  /** protobuf container for the agent settings */
  dsopp::sensors::proto::AgentSettings agent_settings_;
  /** container for the odometry track */
  OdometryTrack odometry_track_;
};
}  // namespace dsopp::track::storage

#endif  // DSOPP_TRACK_STORAGE_HPP
