#include "visualizer/visualizer_track_output_interface.hpp"

#include "energy/motion/se3_motion.hpp"
#include "track/active_odometry_track.hpp"
#include "track/active_track.hpp"
#include "track/odometry_track.hpp"
#include "track/track.hpp"
#include "visualizer/local_track.hpp"

namespace dsopp {
namespace output {

template <template <class> class OdometryTrackType, energy::motion::Motion Motion>
VisualizerTrackOutputInterface<OdometryTrackType, Motion>::VisualizerTrackOutputInterface(LocalTrack &local_track)
    : local_track_(local_track) {}

namespace {
template <energy::motion::Motion Motion>
std::vector<size_t> activeFramesIndices(const track::OdometryTrack<Motion> &) {
  return {};
}

template <energy::motion::Motion Motion>
std::vector<size_t> activeFramesIndices(const track::ActiveOdometryTrack<Motion> &track) {
  std::vector<size_t> changed_indices = {};
  for (const auto &frame : track.activeFrames()) {
    changed_indices.push_back(frame->keyframeId());
  }
  return changed_indices;
}

}  // namespace

template <template <class> class OdometryTrackType, energy::motion::Motion Motion>
void VisualizerTrackOutputInterface<OdometryTrackType, Motion>::notify(
    const track::TrackBase<OdometryTrackType, Motion> &track) {
  const auto &odometry_track = track.odometryTrack();
  std::vector<size_t> current_active_frames_indices = activeFramesIndices(odometry_track);
  std::sort(current_active_frames_indices.begin(), current_active_frames_indices.end());

  std::vector<size_t> active_frames_indicies;
  std::merge(current_active_frames_indices.begin(), current_active_frames_indices.end(),
             last_active_frames_indices.begin(), last_active_frames_indices.end(),
             std::back_inserter(active_frames_indicies));
  active_frames_indicies.erase(std::unique(active_frames_indicies.begin(), active_frames_indicies.end()),
                               active_frames_indicies.end());

  last_active_frames_indices = current_active_frames_indices;

  if (local_track_.trackId() == nullptr) {
    local_track_.fromTrack(track);
    return;
  }
  local_track_.updateActiveOdometryFrames(odometry_track, active_frames_indicies, current_active_frames_indices);
}

template <template <class> class OdometryTrackType, energy::motion::Motion Motion>
VisualizerTrackOutputInterface<OdometryTrackType, Motion>::~VisualizerTrackOutputInterface() = default;

template class VisualizerTrackOutputInterface<track::ActiveOdometryTrack, energy::motion::SE3<Precision>>;
template class VisualizerTrackOutputInterface<track::OdometryTrack, energy::motion::SE3<Precision>>;

}  // namespace output
}  // namespace dsopp
