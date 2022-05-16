#include "visualizer/local_odometry_frame.hpp"

#include "track/frames/active_keyframe.hpp"
#include "track/frames/keyframe.hpp"

namespace dsopp {
namespace output {

namespace {
const Eigen::Vector4d kRedColor{1, 0, 0.2, 1};
const Eigen::Vector4d kLightRedColor{1, 0, 0.2, 0.1};
const Eigen::Vector4d kGreenColor{0, 1, 0.35, 1};
const Eigen::Vector4d kLightGreenColor{0, 1, 0.35, 0.1};
}  // namespace
template <energy::motion::Motion Motion>
LocalOdometryFrame::LocalOdometryFrame(const track::Keyframe<Motion> &frame) : LocalFrame(frame, kRedColor) {
  frame.points(frame_points_storage);

  const Eigen::Vector4d kAttachedToFrameColor = kLightRedColor;
  for (const auto &attached_frame : frame.attachedFrames()) {
    attached_frames.emplace_back(*attached_frame, kAttachedToFrameColor);
  }
}

template <energy::motion::Motion Motion>
LocalOdometryFrame::LocalOdometryFrame(const track::ActiveKeyframe<Motion> &frame)
    : LocalFrame(frame, frame.isMarginalized() ? kRedColor : kGreenColor) {
  frame.points(frame_points_storage);

  const Eigen::Vector4d kAttachedToMarginalizedFrameColor = kLightRedColor;
  const Eigen::Vector4d kAttachedToActiveFrameColor = kLightGreenColor;

  for (const auto &attached_frame : frame.attachedFrames()) {
    attached_frames.emplace_back(
        *attached_frame, frame.isMarginalized() ? kAttachedToMarginalizedFrameColor : kAttachedToActiveFrameColor);
  }
}

#ifndef BUILDING_DOCS
template LocalOdometryFrame::LocalOdometryFrame(const track::Keyframe<energy::motion::SE3<Precision>> &);
template LocalOdometryFrame::LocalOdometryFrame(const track::ActiveKeyframe<energy::motion::SE3<Precision>> &);

#endif /* BUILDING_DOCS */
}  // namespace output
}  // namespace dsopp
