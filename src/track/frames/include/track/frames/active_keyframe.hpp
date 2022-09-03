#ifndef DSOPP_SRC_TRACK_ACTIVE_FRAME_HPP_
#define DSOPP_SRC_TRACK_ACTIVE_FRAME_HPP_

#include <opencv2/core.hpp>

#include "common/settings.hpp"
#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"
#include "features/camera/camera_features.hpp"
#include "features/camera/pixel_data_frame.hpp"
#include "sensors/camera_calibration/mask/camera_mask.hpp"
#include "track/frames/frame.hpp"
#include "track/frames/slam_internal_tracking_frame.hpp"

#include <memory>

namespace dsopp {
namespace features {
class TrackingFeaturesFrame;
class PatternPatch;
}  // namespace features
namespace track {
namespace storage {
struct PointsStorage;
}
template <energy::motion::MotionProduct MotionProduct>
class FrameConnection;
namespace landmarks {
class ImmatureTrackingLandmark;
class ActiveTrackingLandmark;
}  // namespace landmarks
/**
 * activated state of the agent in the given moment. Contains activated landmarks.
 * @tparam Motion Motion type
 */
template <energy::motion::Motion Motion>
class ActiveKeyframe : public Frame<Motion> {
 public:
  /** immature landmark activation status */
  enum struct ImmatureLandmarkActivationStatus : uint8_t {
    kActivate = 0, /**< activate immature landmark. */
    kSkip = 1,     /**< skip immature landmark and try activate later. */
    kDelete = 2    /**< delete immature landmark. */
  };
  /** shortcut for the immature landmark activation statuses per frame */
  using ImmatureLandmarkActivationStatusesPerSensor = std::map<size_t, std::vector<ImmatureLandmarkActivationStatus>>;
  /** shortcut for data from sensors */
  using Pyramids = std::map<size_t, features::Pyramid>;
  /** shortcut for masks from sensors */
  using PyramidsOfMasks = std::map<size_t, features::CameraFeatures::PyramidOfMasks>;
  /** shortcut for immature landmarks frame */
  using ImmatureLandmarksFrame = std::map<size_t, std::vector<landmarks::ImmatureTrackingLandmark>>;
  /** shortcut for active landmarks frame */
  using ActiveLandmarksFrame = std::map<size_t, std::vector<landmarks::ActiveTrackingLandmark>>;
  /**
   * @param id frame id
   * @param keyframe_id id among other keyframes in the track
   * @param timestamp time of the capture
   * @param tWorldAgent pose of the agent
   * @param exposure_time exposure time
   * @param affine_brightness affine brightness
   */
  ActiveKeyframe(size_t id, size_t keyframe_id, time timestamp, const Motion &tWorldAgent,
                 const Precision exposure_time = 1,
                 const Eigen::Vector<Precision, 2> &affine_brightness = Eigen::Vector<Precision, 2>::Zero());
  /**
   * @return id among other keyframes in the track
   */
  size_t keyframeId() const;
  /**
   * set transformation from agent's coordinate system to world's
   * @param tWorldAgent new pose of the frame
   */
  void setTWorldAgent(const Motion &tWorldAgent);
  /**
   * set affine brightness
   * @param affine_brightness new affine brightness
   */
  void setAffineBrightness(const Eigen::Vector<Precision, 2> &affine_brightness);
  /**
   * attach frames to this keyframes with a relative transformation
   * @param id id of the frame
   * @param timestamp timestamp of the frame
   * @param tKeyframeAgent transformation from attached frame to keyframe
   * @param exposure_time exposure time
   * @param affine_brightness affine brightness of attached frame
   * @param mean_square_optical_flow mean square optical flow from keyframe to frame
   * @param mean_square_optical_flow_without_rotation mean square optical flow without rotation from keyframe to frame
   * @param pose_rmse mean frame energy relative to a keyframe
   * @param reliable pose estimation was reliable
   */
  void attachTrackingFrame(size_t id, time timestamp, const typename Motion::Product &tKeyframeAgent,
                           const Precision exposure_time = 1,
                           const Eigen::Vector<Precision, 2> &affine_brightness = Eigen::Vector<Precision, 2>::Zero(),
                           Precision mean_square_optical_flow = 0,
                           Precision mean_square_optical_flow_without_rotation = 0, Precision pose_rmse = 0,
                           bool reliable = true);
  /**
   * @return frames attached to this keyframe
   */
  const std::vector<std::unique_ptr<SLAMInternalTrackingFrame<Motion>>> &attachedFrames() const;
  /**
   * get last attached frame
   * @return last ``attached_frames_``
   */
  SLAMInternalTrackingFrame<Motion> &lastAttachedFrame();
  /**
   * adds tracking features of the sensor
   * @param sensor id of the sensor
   * @param features coordinates and directions of features
   */
  void pushImmatureLandmarks(
      size_t sensor, const std::vector<std::pair<Eigen::Vector<Precision, 2>, Eigen::Vector<Precision, 3>>> &features);

  /**
   * add semantics data to frame
   * @param sensor sensor id
   * @param semantics_data semantic data
   */
  void pushSemanticsData(size_t sensor, std::unique_ptr<cv::Mat> &&semantics_data);
  /**
   * @param sensor id of the sensor
   * @return semantic segmentation of the sensor
   */
  const cv::Mat *semanticsData(size_t sensor) const;

  /**
   * @param[out] frame_points_storage all points in the frame for the visualization
   */
  void points(std::map<std::string, storage::PointsStorage> &frame_points_storage) const;

  /**
   * const method to get const reference to active landmark
   * @param sensor sensor id
   * @param idx index
   * @return const ref to landmark
   */
  const landmarks::ActiveTrackingLandmark &getActiveLandmark(size_t sensor, size_t idx) const;
  /**
   * const method to get const reference to immature landmark
   * @param sensor sensor id
   * @param idx index
   * @return const ref to landmark
   */
  const landmarks::ImmatureTrackingLandmark &getImmatureLandmark(size_t sensor, size_t idx) const;
  /**
   * method to get const reference to active landmark
   * @param sensor sensor id
   * @param idx index
   * @return ref to landmark
   */
  landmarks::ActiveTrackingLandmark &getActiveLandmark(size_t sensor, size_t idx);
  /**
   * method to get const reference to immature landmark
   * @param sensor sensor id
   * @param idx index
   * @return ref to landmark
   */
  landmarks::ImmatureTrackingLandmark &getImmatureLandmark(size_t sensor, size_t idx);
  /**
   * method to get const reference to vector of immature landmarks
   * @param sensor sensor id
   * @return const reference to immature landmarks from `sensor` sensor
   */
  const std::vector<landmarks::ImmatureTrackingLandmark> &immatureLandmarks(size_t sensor) const;
  /**
   * method to get const reference to vector of active landmarks
   * @param sensor sensor id
   * @return const reference to active landmarks from `sensor` sensor
   */
  const std::vector<landmarks::ActiveTrackingLandmark> &activeLandmarks(size_t sensor) const;
  /**
   * @return sensors id that have points
   */
  std::vector<size_t> sensors() const;
  /**
   * marginalizes key frame
   */
  void marginalize();
  /**
   * clear resources of a marginalized keyframe
   */
  void unloadResources();
  /**
   * try to activate all immature points
   * @return number of new activated points on each sensor
   */
  std::map<size_t, size_t> activateLandmarks();
  /**
   * @return true if marginalized
   */
  bool isMarginalized() const;
  /**
   * method to get pyramid level of the data
   * @param sensor id of the sensor
   * @param level level to get
   * @return pyramid level
   */
  const features::PixelMap<1> &getLevel(const size_t sensor, size_t level) const;
  /**
   * @param sensor sensor id
   * @param pyramid pyramid of images
   */
  void pushPyramid(const size_t sensor, features::Pyramid &&pyramid);
  /**
   * @param sensor sensor id
   * @param pyramid_of_masks pyramid of masks
   */
  void pushPyramidOfMasks(const size_t sensor, features::CameraFeatures::PyramidOfMasks &&pyramid_of_masks);
  /**
   * method to get mask from the sensor
   * @param sensor id of the sensor
   * @param level level to get
   * @return mask from the sensor
   */
  const sensors::calibration::CameraMask &getMask(const size_t sensor, size_t level) const;
  /**
   * method to get all masks
   * @return masks
   */
  const PyramidsOfMasks &pyramidsOfMasks() const;
  /**
   * method to get all pyramids frame the frame
   * @return pyramids
   */
  const Pyramids &pyramids() const;
  /*
   *
   * @return connections of frame indexed by target id
   */
  const std::map<size_t, FrameConnection<typename Motion::Product> *> &connections() const;
  /**
   * @param target_keyframe_id target frame id
   * @return frame connection of current frame with queried frame
   */
  FrameConnection<typename Motion::Product> &getConnection(size_t target_keyframe_id) const;
  /**
   * adds connection reference to frame
   * @param target_keyframe_id  target frame id
   * @param connection connection to add
   */
  void addConnection(size_t target_keyframe_id, FrameConnection<typename Motion::Product> *connection);
  /**
   * apply activation statuses to immature landmarks
   * @param statuses new statuses
   * @return number of new activated landmarks on each sensor
   */
  std::map<size_t, size_t> applyImmatureLandmarkActivationStatuses(
      const ImmatureLandmarkActivationStatusesPerSensor &statuses);
  ~ActiveKeyframe();

 private:
  /** id among other keyframes in the track */
  size_t keyframe_id_;
  /** frames attached to the current keyframe */
  std::vector<std::unique_ptr<SLAMInternalTrackingFrame<Motion>>> attached_frames_;
  /** container to store sensors data */
  Pyramids pyramids_;
  /** container to store frame embeddings */
  // contact Roadly INC for this functionality std::map<size_t, std::unique_ptr<features::PixelMap<kChannelsNumber>>>
  // frame_embeddings_;
  /** Camera mask (may contain dynamic objects) */
  PyramidsOfMasks pyramids_of_masks_;
  /** frame connections indexed by target frame id*/
  std::map<size_t, FrameConnection<typename Motion::Product> *> connections_;
  /** container to store landmarks */
  ImmatureLandmarksFrame immature_landmarks_;
  /** container to store landmarks */
  ActiveLandmarksFrame active_landmarks_;
  /** true if marginalized */
  bool is_marginalized_;
  /** semantic data */
  std::map<size_t, std::unique_ptr<cv::Mat>> semantics_data_;
};

}  // namespace track
}  // namespace dsopp

#endif  // DSOPP_SRC_TRACK_ACTIVE_FRAME_HPP_
