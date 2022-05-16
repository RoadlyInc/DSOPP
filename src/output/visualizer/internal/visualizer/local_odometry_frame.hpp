#ifndef DSOPP_LOCAL_ODOMETRY_FRAME_HPP
#define DSOPP_LOCAL_ODOMETRY_FRAME_HPP

#include "visualizer/local_frame.hpp"

#include <map>
#include <string>
#include <vector>

#include "visualizer/buffer_storage.hpp"
#include "visualizer/local_tracking_frame.hpp"
#include "visualizer/points_storage.hpp"

namespace dsopp {
namespace track {
template <energy::motion::Motion Motion>
class Keyframe;
template <energy::motion::Motion Motion>
class ActiveKeyframe;
}  // namespace track

namespace output {
/**
 * Struct to store the local copy of the odometry frame.
 */
struct LocalOdometryFrame : public LocalFrame {
  /**
   * create a local copy of the keyframe
   * @param frame keyframe to copy
   */
  template <energy::motion::Motion Motion>
  LocalOdometryFrame(const track::Keyframe<Motion> &frame);
  /**
   * create a local copy of the active keyframe
   * @param frame active keyframe to copy
   */
  template <energy::motion::Motion Motion>
  LocalOdometryFrame(const track::ActiveKeyframe<Motion> &frame);
  /** points storage for the visualization */
  std::map<std::string, PointsStorage> frame_points_storage;
  /** tracking frames attached to keyframe */
  std::vector<LocalTrackingFrame> attached_frames;
  /** true when the frame buffers have been initialized */
  bool are_buffers_initialized = false;
  /** true if the frame should be removed */
  bool should_be_removed = false;
  /** points storage buffer for the visualization */
  std::map<std::string, BufferStorage> frame_buffer_storage;
};
}  // namespace output
}  // namespace dsopp

#endif  // DSOPP_LOCAL_ODOMETRY_FRAME_HPP
