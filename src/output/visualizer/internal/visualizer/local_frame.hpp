#ifndef DSOPP_VISUALIZER_LOCAL_FRAME_HPP
#define DSOPP_VISUALIZER_LOCAL_FRAME_HPP

#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include "common/settings.hpp"
#include "common/time/time.hpp"
#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"

namespace dsopp {
namespace output {

namespace tools {

const std::optional<Sophus::SE3d> getDebugCamera(const energy::motion::SE3<Precision> &);
}  // namespace tools

/**
 * Struct to store the local copy of the odometry frame.
 */
struct LocalFrame {
  /**
   * create a local copy of the keyframe
   * @param frame keyframe to copy
   * @param _color color for visualization
   */
  template <class FrameType>
  LocalFrame(const FrameType &frame, const Eigen::Vector4d &_color)
      : timestamp(frame.timestamp()),
        t_w_agent(frame.tWorldAgent().se3().template cast<double>()),
        debug_camera(tools::getDebugCamera(frame.tWorldAgent())),
        color(_color) {}

  /** timestamp of the frame */
  const time timestamp;
  /** Agent to world transformation */
  const Sophus::SE3d t_w_agent;
  /** additional camera for debug visualization */
  const std::optional<Sophus::SE3d> debug_camera;  // TODO: support generalized decorators
  /** color of the frame */
  const Eigen::Vector4d color;
};
}  // namespace output
}  // namespace dsopp

#endif  // DSOPP_VISUALIZER_LOCAL_FRAME_HPP
