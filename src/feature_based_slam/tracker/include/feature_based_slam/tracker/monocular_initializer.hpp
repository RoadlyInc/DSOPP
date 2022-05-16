#ifndef DSOPP_FEATURE_BASED_SLAM_TRACKER_MONOCULAR_INITIALIZER_HPP_
#define DSOPP_FEATURE_BASED_SLAM_TRACKER_MONOCULAR_INITIALIZER_HPP_

#include "feature_based_slam/features/distinct_features_extractor.hpp"
#include "feature_based_slam/features/distinct_features_frame.hpp"
#include "feature_based_slam/tracker/initializer.hpp"
#include "features/camera/camera_features.hpp"
#include "features/camera/pixel_data_frame.hpp"
#include "features/camera/tracking_features_frame.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"

namespace dsopp::feature_based_slam::tracker {
/**
 * \brief class for Initialize monocular reconstruction. Initializer cannot exist without tracker.
 *
 * Initialized via Essential Matrix + triangulation + PnP
 * has build-in rotation ransac solver to exclude frames with small translations from optimization
 *
 *  @tparam Calibration camera calibration type
 */
template <energy::model::Model Model>
class MonocularInitializer : public Initializer {
 public:
  /**
   * Constructor to create initializer
   * @param track track which contain all frames and landmarks
   * @param sensor_id sensor id for initialization
   * @param options All constants which will be used in the feature based slam algorithms and optimizations
   * @param feature_extractor feature extractor
   * @param initializer_keyframe_strategy this object decides when to start initialization
   * @param model camera model
   */
  MonocularInitializer(
      track::Track &track, const size_t sensor_id, const features::DistinctFeaturesExtractor &feature_extractor,
      std::unique_ptr<initialization_strategy::InitializerKeyframeStrategy> &&initializer_keyframe_strategy,
      const Options &options, std::unique_ptr<Model> &&model);

  void tick(const sensors::SynchronizedFrame &frame_data, const size_t number_of_threads) override;

  virtual Eigen::VectorX<Precision> cameraIntrinsics() const override;

  ~MonocularInitializer() override;

 protected:
  /** camera model */
  std::unique_ptr<Model> model_;
  /** this object decides when to start initialization */
  std::unique_ptr<initialization_strategy::InitializerKeyframeStrategy> initializer_keyframe_strategy_;
};
}  // namespace dsopp::feature_based_slam::tracker

#endif  // DSOPP_FEATURE_BASED_SLAM_TRACKER_MONOCULAR_INITIALIZER_HPP_
