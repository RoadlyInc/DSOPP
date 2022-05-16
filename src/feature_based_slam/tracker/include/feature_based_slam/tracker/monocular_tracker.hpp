#ifndef DSOPP_FEATURE_BASED_SLAM_TRACKER_MONOCULAR_TRACKER_HPP
#define DSOPP_FEATURE_BASED_SLAM_TRACKER_MONOCULAR_TRACKER_HPP

#include <functional>

#include "feature_based_slam/features/correspondences_finder.hpp"
#include "feature_based_slam/initialization_strategy/initializer_keyframe_strategy.hpp"
#include "feature_based_slam/track/track.hpp"
#include "feature_based_slam/tracker/initializer.hpp"
#include "feature_based_slam/tracker/tracker.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"

namespace dsopp {
namespace feature_based_slam {
namespace tracker {
class Initializer;
template <energy::model::Model Model>
/**
 * Feature based slam tracker
 */
class MonocularTracker : public Tracker {
 public:
  /**
   * Creates features based slam tracker,
   * @param sensor_id sensor id for initialization
   * @param feature_extractor feature extractor
   * @param initializer_fabric initializer fabric
   * @param model camera model. Can be changed during initialization/tracking. Can be nullptr and founded automatically.
   * @param options all constants which will be used in the feature based slam algorithms and optimizations
   * */
  MonocularTracker(
      const size_t sensor_id, std::unique_ptr<features::DistinctFeaturesExtractor> &&feature_extractor,
      std::function<std::unique_ptr<Initializer>(track::Track &, const features::DistinctFeaturesExtractor &)>
          &initializer_fabric,
      std::unique_ptr<Model> &&model, const Options &options = Options());

  void tick(const sensors::SynchronizedFrame &frame_data, const size_t number_of_threads) override;

  virtual Eigen::VectorX<Precision> cameraIntrinsics() const override;

  ~MonocularTracker() override;

 private:
  /** camera model. Can be changed during initialization/tracking */
  std::unique_ptr<Model> model_;
};

}  // namespace tracker
}  // namespace feature_based_slam
}  // namespace dsopp

#endif  // DSOPP_FEATURE_BASED_SLAM_TRACKER_MONOCULAR_TRACKER_HPP
