#ifndef DSOPP_SRC_OUTPUT_VISUALIZER_TRACK_OUTPUT_INTERFACE_HPP
#define DSOPP_SRC_OUTPUT_VISUALIZER_TRACK_OUTPUT_INTERFACE_HPP

#include "output_interfaces/track_output_interface.hpp"
#include "visualizer/visualizer.hpp"
#include "visualizer/visualizer_output_interface.hpp"

namespace dsopp {
namespace output {
class LocalTrack;

/**
 * \brief Tracks visualizer.
 *
 * 3D display of tracks and point clouds.
 */
template <template <class> class OdometryTrackType, energy::motion::Motion Motion>
class VisualizerTrackOutputInterface : public TrackOutputInterface<OdometryTrackType, Motion>,
                                       public VisualizerOutputInterface {
 public:
  /**
   * create new VisualizerOutputInterface
   * @param local_track local copy of the track to be modified
   */
  VisualizerTrackOutputInterface(LocalTrack &local_track);
  /**
   * notify that some frames changed in the track
   * @param track changed track
   */
  void notify(const track::TrackBase<OdometryTrackType, Motion> &track) override;

  ~VisualizerTrackOutputInterface() override;

 protected:
  /** local copy of the track */
  LocalTrack &local_track_;
  /** set of active frame indices from previous notification*/
  std::vector<size_t> last_active_frames_indices = {};
};

}  // namespace output
}  // namespace dsopp

#endif  // DSOPP_SRC_OUTPUT_VISUALIZER_OUTPUT_INTERFACE_HPP
