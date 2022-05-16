#include "visualizer/local_tracking_frame.hpp"

#include "common/settings.hpp"

#include "track/frames/slam_internal_tracking_frame.hpp"
#include "track/frames/tracking_frame.hpp"

namespace dsopp {
namespace output {
template <class TrackingFrameType>
LocalTrackingFrame::LocalTrackingFrame(const TrackingFrameType &frame, const Eigen::Vector4d &_color)
    : LocalFrame(frame, _color) {}

#ifndef BUILDING_DOCS
template LocalTrackingFrame::LocalTrackingFrame(const track::TrackingFrame<energy::motion::SE3<Precision>> &,
                                                const Eigen::Vector4d &);
template LocalTrackingFrame::LocalTrackingFrame(
    const track::SLAMInternalTrackingFrame<energy::motion::SE3<Precision>> &, const Eigen::Vector4d &);

#endif /* BUILDING_DOCS */
}  // namespace output
}  // namespace dsopp
