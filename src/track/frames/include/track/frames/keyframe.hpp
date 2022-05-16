#ifndef DSOPP_KEYFRAME_HPP
#define DSOPP_KEYFRAME_HPP

#include "common/settings.hpp"
#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"
#include "frame.pb.h"
#include "track/frames/frame.hpp"

namespace dsopp {
namespace track {

template <energy::motion::Motion Motion>
class TrackingFrame;
namespace landmarks {
class TrackingLandmark;
}
/**
 * result state of the agent in the given moment. Contains attached non-keyframes and marginalized landmarks.
 * @tparam Motion Motion type
 */
template <energy::motion::Motion Motion>
class Keyframe : public Frame<Motion> {
 public:
  /** shortcut for result landmarks frame */
  using LandmarksFrame = std::map<size_t, std::vector<landmarks::TrackingLandmark>>;
  /**
   * @param id frame id
   * @param keyframe_id id among other keyframes in the track
   * @param timestamp time of the capture
   * @param tWorldAgent pose of the agent
   * @param affine_brightness affine brightness
   * @param landmarks landmarks attached to the frame
   */
  Keyframe(size_t id, size_t keyframe_id, time timestamp, const Motion &tWorldAgent,
           const Eigen::Vector<Precision, 2> &affine_brightness, LandmarksFrame &&landmarks);
  /**
   * creates frame from the protobuf container
   * @param proto protobuf container
   */
  explicit Keyframe(const proto::Keyframe &proto);
  /**
   * @return id among other keyframes in the track
   */
  size_t keyframeId() const;
  /**
   * @param[out] frame_points_storage all points in the frame for the visualization
   */
  void points(std::map<std::string, storage::PointsStorage> &frame_points_storage) const;
  /**
   * creates protobuf container from the object
   * @return protobuf container
   */
  proto::Keyframe proto() const;
  /**
   * Attaches frames to this keyframes with a relative transformation
   * @param id id of the frame
   * @param timestamp timestamp of the frame
   * @param tKeyframeAgent transformation from attached frame to keyframe
   * @param affine_brightness affine brightness of attached frame
   */
  void attachTrackingFrame(size_t id, time timestamp, const typename Motion::Product &tKeyframeAgent,
                           const Eigen::Vector<Precision, 2> &affine_brightness);
  /**
   * @return frames attached to this keyframe
   */
  const std::vector<std::unique_ptr<TrackingFrame<Motion>>> &attachedFrames() const;
  ~Keyframe();

  /**
   * get last attached frame
   * @return last ``attached_frames_``
   */
  TrackingFrame<Motion> &lastAttachedFrame();

  /**
   * method to get const refernce to vetor of landmakrs
   * @param sensor sensor id
   * @return const reference to tracking landmarks
   */
  const std::vector<landmarks::TrackingLandmark> &landmarks(size_t sensor) const;

 private:
  /** id among other keyframes in the track */
  size_t keyframe_id_;
  /** frames attached to the current keyframe */
  std::vector<std::unique_ptr<TrackingFrame<Motion>>> attached_frames_;
  /** container to store landmarks */
  LandmarksFrame landmarks_;
};

}  // namespace track
}  // namespace dsopp

#endif  // DSOPP_KEYFRAME_HPP
