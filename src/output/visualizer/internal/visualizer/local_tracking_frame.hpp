#ifndef DSOPP_LOCAL_TRACKING_FRAME_HPP
#define DSOPP_LOCAL_TRACKING_FRAME_HPP

#include "visualizer/local_frame.hpp"

#include <Eigen/Dense>

#include "common/time/time.hpp"

namespace dsopp {
namespace output {
/**
 * Struct to store the local copy of the tracking(attached to keyframe) frame
 */
struct LocalTrackingFrame : public LocalFrame {
  /**
   * create a local copy of the tracking frame
   * @param frame tracking frame to copy
   * @param _color tracking frame color
   */
  template <class TrackingFrameType>
  LocalTrackingFrame(const TrackingFrameType &frame, const Eigen::Vector4d &_color);
};
}  // namespace output
}  // namespace dsopp

#endif  // DSOPP_LOCAL_TRACKING_FRAME_HPP
