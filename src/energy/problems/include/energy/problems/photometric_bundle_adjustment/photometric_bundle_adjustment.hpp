#ifndef DSOPP_SRC_ENERGY_FUNCTIONALS_POSE_ALIGNMENT_PROBLEM_HPP_
#define DSOPP_SRC_ENERGY_FUNCTIONALS_POSE_ALIGNMENT_PROBLEM_HPP_

#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"
#include "energy/problems/photometric_bundle_adjustment/frame_parameterization.hpp"
#include "features/camera/pixel_map.hpp"
#include "sensors/camera/camera.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"
#include "track/frames/active_keyframe.hpp"
#include "track/landmarks/tracking_landmark.hpp"

class LinearSystemBenchmarkData;
namespace dsopp {
namespace energy {
namespace problem {

struct DepthMap;
template <typename Scalar, energy::motion::Motion Motion, typename Model, int PatternSize,
          template <int> typename Grid2D, int C, bool FIRST_ESTIMATE_JACOBIANS, bool OPTIMIZE_IDEPTHS>
class BundleAdjustmentPhotometricEvaluationCallback;
template <typename Scalar, energy::motion::Motion Motion, energy::model::Model Model, int PatternSize,
          template <int> typename Grid2D, int C>
class LocalFrame;

/** \brief Concept for all implementations that solve the photometric bundle adjustment.
 *
 * All classes that solve the photometric bundle adjustment using a depth map have to be arguments to this class.
 * @tparam Motion Motion type
 */
template <typename Scalar, energy::motion::Motion Motion, model::Model Model, int PatternSize,
          template <int> typename Grid2D = features::PixelMap, bool OPTIMIZE_POSES = true,
          bool OPTIMIZE_IDEPTHS = false, bool FIRST_ESTIMATE_JACOBIANS = OPTIMIZE_IDEPTHS, int C = 1>
class PhotometricBundleAdjustment {
 public:
  /** shortcut for data from sensors */
  using Pyramids = std::map<size_t, std::vector<features::PixelMap<C>>>;
  /** shortcut for masks from sensors */
  using PyramidsOfMasks = typename track::ActiveKeyframe<Motion>::PyramidsOfMasks;

  /**
   * Constructor to create solver
   * @param estimate_uncertainty estimate uncertainty of poses and idepths after optimization
   */
  PhotometricBundleAdjustment(bool estimate_uncertainty = false);
  /**
   * Push a new frame in the solver. The new frame should have a later timestamp than the previous frames.
   * A local copy of the frame will be created for optimization, the frame will not be changed.
   *
   * @param frame new frame to use in optimization
   * @param level level of the pyramid which will be added to the solver
   * @param model camera model for frame
   * @param frame_parameterization constraint on frame position
   */
  virtual void pushFrame(const track::ActiveKeyframe<Motion> &frame, size_t level, const Model &model,
                         FrameParameterization frame_parameterization = FrameParameterization::kFree);
  /**
   * Push a frame with the given depth map. Used to estimate the next pose.
   *
   * @param timestamp timestamp of the frame
   * @param t_world_agent initial pose of the frame
   * @param pyramids pyramids of the data from sensors
   * @param masks masks from the sensors
   * @param depths_maps depth maps for each sensor
   * @param exposure_time exposure time
   * @param affine_brightness affine brightness
   * @param level level of the pyramid which will be added to the solver
   * @param model camera model for frame
   * @param frame_parameterization constraint on frame position
   */
  void pushFrame(time timestamp, const Motion &t_world_agent, const Pyramids &pyramids,
                 const std::map<size_t, const sensors::calibration::CameraMask &> &masks,
                 const std::map<size_t, std::vector<DepthMap>> &depths_maps, const Precision exposure_time,
                 const Eigen::Vector2<Precision> &affine_brightness, size_t level, const Model &model,
                 FrameParameterization frame_parameterization = FrameParameterization::kFree);

  /**
   * Push a frame with a vector of landmarks. Used to estimate the next pose.
   *
   * @param timestamp timestamp of the frame
   * @param t_world_agent initial pose of the frame
   * @param pyramids pyramids of the data from sensors
   * @param masks masks from the sensors
   * @param tracking_landmarks map of sensor_id -> tracking landmark vector
   * @param exposure_time exposure time
   * @param affine_brightness affine brightness
   * @param model camera model
   * @param level level of the pyramid which will be added to the solver
   * @param level_resize_ratio resize ratio of level image
   * @param frame_parameterization constraint on frame position
   */
  void pushFrame(time timestamp, const Motion &t_world_agent, const Pyramids &pyramids,
                 const std::map<size_t, const sensors::calibration::CameraMask &> &masks,
                 const std::map<size_t, std::vector<track::landmarks::TrackingLandmark>> &tracking_landmarks,
                 const Precision exposure_time, const Eigen::Vector2<Precision> &affine_brightness, const Model &model,
                 size_t level, Precision level_resize_ratio,
                 FrameParameterization frame_parameterization = FrameParameterization::kFree);

  /**
   * Push a new local frame in the solver. The new frame should have a later timestamp than the previous frames.
   *
   * @param timestamp timestamp of the frame
   * @param t_world_agent_init initial pose of the frame
   * @param pyramids pyramids of the data from sensors
   * @param masks masks from the sensors
   * @param exposure_time exposure time
   * @param affine_brightness affine brightness
   * @param level level of the pyramid which will be added to the solver
   * @param model camera model for frame
   * @param frame_parameterization constraint on frame position
   */
  void pushFrame(time timestamp, const Motion &t_world_agent_init, const Pyramids &pyramids,
                 const std::map<size_t, const sensors::calibration::CameraMask &> &masks, const Precision exposure_time,
                 const Eigen::Vector2<Precision> &affine_brightness, size_t level, const Model &model,
                 FrameParameterization frame_parameterization = FrameParameterization::kFree);
  /**
   * Push information in the frame from the local copy.
   *
   * @param frame frame to update
   */
  void updateFrame(track::ActiveKeyframe<Motion> &frame);
  /**
   * update information in local frame
   *
   * @param frame frame to get updates from
   */
  virtual void updateLocalFrame(const track::ActiveKeyframe<Motion> &frame);
  /**
   * Get the pose of the local frame with the given timestamp
   *
   * @param timestamp given timestamp
   * @return pose of the local frame with the given timestamp and identity, if timestamp doesn't exist.
   */
  Motion getPose(time timestamp) const;
  /**
   * Get the affine brightness of the local frame with the given timestamp
   *
   * @param timestamp given timestamp
   * @return affine brightness of the local frame with the given timestamp and zero, if timestamp doesn't exist.
   */
  Eigen::Vector2<Precision> getAffineBrightness(time timestamp) const;

  virtual ~PhotometricBundleAdjustment();
  /**
   * friend class for testing purposes
   */
  friend class ::LinearSystemBenchmarkData;

  /** function to solve the problem in derived classes (CeresPhotometricBundleAdjustment or
   * EigenPhotometricBundleAdjustment)
   * @param number_of_threads number of threads for ceres optimizer
   * @return final cost after optimization
   */
  virtual Precision solve(const size_t number_of_threads) = 0;

 protected:
  /** estimate jacobians in the point of linearization. For details see DSO text between eq 13 and 14 */
  void firstEstimateJacobians();
  /** update point statuses
   * @param minimum_valid_reprojections_num minimum valid number of reporjection in frames to keep landmark as an inlier
   * @param sigma_huber_loss sigma huber loss
   * */
  void updatePointStatuses(const size_t minimum_valid_reprojections_num, const Scalar sigma_huber_loss);
  /**
   * change linearization point for parameters that are not yet a part of a marginalization term
   */
  void relinearizeSystem();
  /**
   * get local frame by frame id
   * @param id id
   * @return frame
   */
  LocalFrame<Scalar, Motion, Model, PatternSize, Grid2D, C> *getLocalFrame(const int id) const;
  /**
   * get local frame by frame timestamp
   * @param timestamp timestamp
   * @return frame
   */
  LocalFrame<Scalar, Motion, Model, PatternSize, Grid2D, C> *getLocalFrame(time timestamp) const;
  /** vector of local copies of frames to optimize */
  std::deque<std::unique_ptr<LocalFrame<Scalar, Motion, Model, PatternSize, Grid2D, C>>> frames_;
  /** estimate uncertainty of poses and idepths after optimization */
  bool estimate_uncertainty_;
};

}  // namespace problem
}  // namespace energy
}  // namespace dsopp

#endif  // DSOPP_SRC_ENERGY_FUNCTIONALS_POSE_ALIGNMENT_PROBLEM_HPP_
