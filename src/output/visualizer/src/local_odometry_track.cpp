#include "visualizer/local_odometry_track.hpp"

#include "track/active_odometry_track.hpp"
#include "track/odometry_track.hpp"

namespace dsopp {
namespace output {
template <energy::motion::Motion Motion>
LocalOdometryTrack::LocalOdometryTrack(const track::OdometryTrack<Motion> &track) {
  for (const auto &frame : track.keyframes()) {
    frames.emplace_back(*frame);
  }
}

template <energy::motion::Motion Motion>
LocalOdometryTrack::LocalOdometryTrack(const track::ActiveOdometryTrack<Motion> &track) {
  for (const auto &marginalized_frame : track.marginalizedFrames()) {
    frames.emplace_back(*marginalized_frame);
  }
  for (const auto &active_frame : track.activeFrames()) {
    frames.emplace_back(*active_frame);
  }
}

#ifndef BUILDING_DOCS
template LocalOdometryTrack::LocalOdometryTrack(const track::OdometryTrack<energy::motion::SE3<Precision>> &);
template LocalOdometryTrack::LocalOdometryTrack(const track::ActiveOdometryTrack<energy::motion::SE3<Precision>> &);
#endif /* BUILDING_DOCS */
}  // namespace output
}  // namespace dsopp
