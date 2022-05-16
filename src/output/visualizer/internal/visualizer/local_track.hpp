#ifndef DSOPP_LOCAL_TRACK_HPP
#define DSOPP_LOCAL_TRACK_HPP

#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"
#include "visualizer/local_odometry_track.hpp"
#include "visualizer/output_parameters.hpp"
#include "visualizer/renderable.hpp"

namespace dsopp {
namespace track {
template <template <class> class OdometryTrackType, energy::motion::Motion Motion>
class TrackBase;
class GnssTrack;
class ECEFPoses;
}  // namespace track

namespace output {
/**
 * Struct to store the local copy of the track.
 */
class LocalTrack : public Renderable {
 public:
  /**
   * create a local copy of the track
   * @param track track to copy
   */
  template <template <class> class OdometryTrackType, energy::motion::Motion Motion>
  void fromTrack(const track::TrackBase<OdometryTrackType, Motion> &track);
  /**
   * Construct a new Local Track object
   * @param output_parameters visualization parameters
   */
  LocalTrack(const OutputParameters &output_parameters);
  /** locks track for read and write */
  void render() override;
  /** @return track id */
  const void *trackId() const;
  /**
   * get position of the camera in the visualizer
   *
   * @param frame_id id of the queried frame
   * @return position of the camera
   */
  const Eigen::Vector3d getCameraPosition(size_t frame_id) const;
  template <typename OdometryTrack>
  /**
   * updates changed frames
   * @param odometry_track reference track
   * @param active_frames_indicies ask maxim TODO:fix HDMRD-147
   * @param current_active_frames_indicies ask maxim TODO:fix HDMRD-147
   */
  void updateActiveOdometryFrames(const OdometryTrack &odometry_track,
                                  const std::vector<size_t> &active_frames_indicies,
                                  const std::vector<size_t> &current_active_frames_indicies);

  void setParameters(const OutputParameters &output_parameters) override;

  /**
   * Set point color, unique for all tracks
   * @param unique_color default track-specific point color
   */
  void setUniqueColor(const Eigen::Vector3<uint8_t> &unique_color);

 private:
  /** parameters for visualization */
  OutputParameters output_parameters_;
  /** pointer to the track (used as an id)*/
  const void *track_id_;
  /** odometry track */
  LocalOdometryTrack odometry_track_;
  /** mutex to lock storage while it's being changed */
  mutable std::mutex mutex_;
  /** Track-unique color for points */
  Eigen::Vector3<uint8_t> unique_point_color_;
};
}  // namespace output
}  // namespace dsopp

#endif  // DSOPP_LOCAL_TRACK_HPP
