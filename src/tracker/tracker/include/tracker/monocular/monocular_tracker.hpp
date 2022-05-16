
#ifndef DSOPP_MONOCULAR_TRACKER_HPP
#define DSOPP_MONOCULAR_TRACKER_HPP

#include "common/settings.hpp"
#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"
#include "feature_based_slam/features/distinct_features_extractor.hpp"
#include "feature_based_slam/features/distinct_features_frame.hpp"
#include "features/camera/pixel_data_frame.hpp"
#include "features/camera/pixel_map.hpp"
#include "features/camera/tracking_features_extractor.hpp"
#include "sensor/synchronized_frame.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"
#include "sensors/camera_providers/camera_data_frame.hpp"
#include "tracker/depth_estimators/depth_estimation.hpp"
#include "tracker/depth_estimators/depth_estimator.hpp"
#include "tracker/tracker.hpp"

namespace dsopp {
namespace energy {
namespace problem {
struct DepthMap;

template <typename Scalar, energy::motion::Motion Motion, model::Model Model, int PatternSize,
          template <int> typename Grid2D, bool OPTIMIZE_POSES, bool OPTIMIZE_IDEPTHS, bool FIRST_ESTIMATE_JACOBIANS,
          int C>
class PhotometricBundleAdjustment;
template <energy::motion::Motion Motion, model::Model Model, int PatternSize, template <int> typename Grid2D, int C>
class PoseAlignment;
}  // namespace problem
}  // namespace energy
namespace marginalization {
template <energy::motion::Motion Motion>
class FrameMarginalizationStrategy;
}  // namespace marginalization
namespace tracker {
template <energy::motion::Motion Motion, energy::model::Model Model, template <int> typename Grid2D, int C, bool REFINE>
class LandmarksActivator;
namespace keyframe_strategy {
template <energy::motion::Motion Motion>
class TrackerKeyframeStrategy;
}  // namespace keyframe_strategy
/**
 * Coarse tracker for monocular case
 */
template <energy::motion::Motion Motion, energy::model::Model Model, typename DepthEstimator,
          template <int> typename Grid2D, bool FRAME_EMBEDDER>
requires tracker::DepthEstimator<DepthEstimator, typename Motion::Product> class MonocularTracker
    : public Tracker<Motion> {
 public:
  /** channels number */
  static constexpr int C = 1;
  /** sigma for the huber loss function */
  static constexpr Precision kHuberLossSigma = 20;

  /** Pattern size in frontend solver */
  static constexpr int kFrontendPatternSize = 1;
  /** Number of channels in frontend solver */
  static constexpr int kFrontendChannelsNumber = 1;

  /**
   * creates Tracker from tracked sensor
   * @param sensor_id sensor identifier for tracking
   * @param calibration camera calibration
   * @param use_imu_prior ``true`` if IMU prior should be used (if available)
   * @param save_images_to_track ``true`` to save images to output track
   * @param tracker_keyframe_strategy tracker strategy how to create the new keyframe
   * @param frame_marginalization_strategy marginalization strategy
   * @param photometric_bundle_adjustment solver for poses and idepthes
   * @param pose_aligner solver for "fast" pose alignment
   * @param landmarks_activator landmarks activator
   */
  MonocularTracker(
      size_t sensor_id, const sensors::calibration::CameraCalibration &calibration, bool use_imu_prior,
      bool save_images_to_track,
      std::unique_ptr<keyframe_strategy::TrackerKeyframeStrategy<Motion>> &&tracker_keyframe_strategy,
      std::unique_ptr<marginalization::FrameMarginalizationStrategy<Motion>> &&frame_marginalization_strategy,
      std::unique_ptr<energy::problem::PhotometricBundleAdjustment<
          Precision, Motion, Model, Pattern::kSize, Grid2D, true, true, true, C>> &&photometric_bundle_adjustment,
      std::unique_ptr<energy::problem::PoseAlignment<Motion, Model, kFrontendPatternSize, Grid2D,
                                                     kFrontendChannelsNumber>> &&pose_aligner,
      std::unique_ptr<LandmarksActivator<Motion, Model, Grid2D, 1, true>> &&landmarks_activator);

  /**
   * method to be called after each synchronization step.
   * @param[in,out] track current track to be updated
   * @param frame_data sensors data from the last tick
   * @param number_of_threads number of threads
   * @param initialization if `true` then tick is called to initialize tracker
   * @param force_keyframe if `true`, keyframe will be created without keyframe strategy
   */
  void tick(track::ActiveTrack<Motion> &track, const sensors::SynchronizedFrame &frame_data,
            const size_t number_of_threads, bool initialization, bool force_keyframe) override;

  void initialize(const feature_based_slam::tracker::Tracker &tracker,
                  const std::vector<std::unique_ptr<sensors::SynchronizedFrame>> &frames,
                  track::ActiveTrack<Motion> &track, const size_t number_of_threads) override;

  bool frameEmbedder() const override;

  ~MonocularTracker() override;

 private:
  /** sensor id for initialization */
  size_t sensor_id_;
  /** camera calibration */
  const sensors::calibration::CameraCalibration &calibration_;
  /** ``true`` when IMU prior should be used (if available) */
  bool use_imu_prior_;
  /** ``true`` to save images to final track file */
  bool save_images_to_track_;
  /** strategy how to create the new keyframe */
  std::unique_ptr<keyframe_strategy::TrackerKeyframeStrategy<Motion>> tracker_keyframe_strategy_;
  /** strategy how to marginalize old keyframes*/
  std::unique_ptr<marginalization::FrameMarginalizationStrategy<Motion>> marginalizer_;
  /** landmarks activator */
  std::unique_ptr<LandmarksActivator<Motion, Model, Grid2D, 1, true>> landmarks_activator_;
  /** reference frame depth maps */
  std::map<size_t, std::vector<energy::problem::DepthMap>> reference_frame_depth_map_;
  /** rmse per pyramid level after last pose estimation. Energy from the finest level is used for keyframe strategy,
   * values from the all levels are used for the re-tracking*/
  std::vector<Precision> rmse_last_pose_estimation_;
  /** Solver for the pose and depth refinement*/
  std::unique_ptr<energy::problem::PhotometricBundleAdjustment<Precision, Motion, Model, Pattern::kSize, Grid2D, true,
                                                               true, true, C>>
      photometric_bundle_adjustment_;
  /** Solver for the pose refinement*/
  std::unique_ptr<energy::problem::PoseAlignment<Motion, Model, kFrontendPatternSize, Grid2D, kFrontendChannelsNumber>>
      pose_aligner_;

  /** only for debug depth map visualization: keeping maximum idepth to be colored as red*/
  Precision maximum_visualized_idepth_ = 0;
};
}  // namespace tracker
}  // namespace dsopp

#endif  // DSOPP_MONOCULAR_TRACKER_HPP
