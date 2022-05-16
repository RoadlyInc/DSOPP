#ifndef DSOPP_FEATURE_BASED_SLAM_TRACK_FRAME_HPP
#define DSOPP_FEATURE_BASED_SLAM_TRACK_FRAME_HPP

#include <memory>
#include <opencv2/core.hpp>

#include "common/settings.hpp"
#include "common/time/time.hpp"
#include "energy/motion/se3_motion.hpp"

namespace dsopp {
namespace feature_based_slam {
namespace track {
/**
 * \brief Class which contains all info to construct tracker`s Frame/Keyframe
 */
struct Frame {
  /**
   * initializer method
   * @param _timestamp timestamp
   * @param _frame_id frame id
   * @param _image image from the camera
   */
  Frame(const time &_timestamp, const size_t _frame_id, cv::Mat _image);
  /** timestamp for initializable frames */
  time timestamp;
  /** frame id */
  size_t frame_id;
  /** frame pose */
  energy::motion::SE3<Precision> t_world_agent;
  /** flag for initialization status of success */
  bool initialized;
  /**
   * \brief contains information about attached frames
   */
  struct AttachedFrame {
    /** timestamp */
    time timestamp;
    /** frame_id */
    size_t frame_id;
    /** rotation from frame to keyframe */
    Sophus::SO3<Precision> rotation_keyframe_frame;
  };
  /** attached droped frames */
  std::vector<AttachedFrame> attached_frames;
  /** image from the camera */
  cv::Mat image;
};

}  // namespace track
}  // namespace feature_based_slam
}  // namespace dsopp

#endif  // DSOPP_FEATURE_BASED_SLAM_TRACK_FRAME_HPP
