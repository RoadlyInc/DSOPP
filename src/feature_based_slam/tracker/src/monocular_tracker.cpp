#include "feature_based_slam/tracker/monocular_tracker.hpp"

#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/camera_model/pinhole/simple_radial.hpp"
#include "feature_based_slam/features/orb.hpp"
#include "feature_based_slam/initialization_strategy/initializer_keyframe_strategy.hpp"
#include "feature_based_slam/initialization_strategy/wait_for_movement_keyframe_strategy.hpp"
#include "feature_based_slam/tracker/add_new_frame.hpp"
#include "feature_based_slam/tracker/estimate_se3_pnp.hpp"
#include "feature_based_slam/tracker/landmark_feature_frame.hpp"
#include "feature_based_slam/tracker/monocular_initializer.hpp"
#include "feature_based_slam/tracker/refine_track.hpp"
#include "feature_based_slam/tracker/remove_outliers.hpp"
#include "feature_based_slam/tracker/triangulate_points.hpp"
#include "sensors/camera_calibration/mask/camera_mask.hpp"

namespace dsopp::feature_based_slam::tracker {

template <energy::model::Model Model>
MonocularTracker<Model>::MonocularTracker(
    const size_t sensor_id, std::unique_ptr<features::DistinctFeaturesExtractor> &&feature_extractor,
    std::function<std::unique_ptr<Initializer>(track::Track &, const features::DistinctFeaturesExtractor &)>
        &initializer_fabric,
    std::unique_ptr<Model> &&model, const Options &options)
    : Tracker(sensor_id, std::move(feature_extractor), initializer_fabric, options), model_(std::move(model)) {}

template <energy::model::Model Model>
void MonocularTracker<Model>::tick(const sensors::SynchronizedFrame &frame_data, const size_t number_of_threads) {
  const size_t kMinProjectionsForTriangulations = 4;
  const Precision kMaxSolverTime = 10._p;
  const size_t kNumberOfFramesToOptimize = 10;

  auto &camera_features = frame_data.cameraFeatures();
  if (!camera_features.contains(sensor_id_)) {
    return;
  }

  if (!initializer_->initialized()) {
    initializer_->tick(frame_data, number_of_threads);
    if (initializer_->initialized()) {
      model_ = std::make_unique<Model>(model_->image_size(), initializer_->cameraIntrinsics(), model_->shutterTime());
    }
    return;
  }

  auto &frame = *camera_features.at(sensor_id_);

  addNewFrame(track_, frame, *model_, *feature_extractor_, this->options_.reprojection_threshold,
              this->options_.rotation_ransac_iterations);
  if (track_.frames.back().initialized) {
    return;
  }

  // estimate pose
  estimateSE3PnP(track_.frames.back().frame_id, track_.landmarks, track_.frames.back().t_world_agent, *model_,
                 options_.pnp_ransac_threshold);
  track_.frames.back().initialized = true;

  // triangulate
  triangulatePoints(track_.landmarks, track_.frames, *model_, kMinProjectionsForTriangulations);

  // refine track
  optimizeTransformations<Model>(track_.landmarks, number_of_threads, model_->image_size(),
                                 model_->intrinsicsParameters(), track_.frames, options_.reprojection_threshold,
                                 kNumberOfFramesToOptimize, kMaxSolverTime);

  // remove outliers
  removeOutliers(track_.landmarks, *model_, track_.frames);
}

template <energy::model::Model Model>
Eigen::VectorX<Precision> MonocularTracker<Model>::cameraIntrinsics() const {
  return model_->intrinsicsParameters();
}

template <energy::model::Model Model>
MonocularTracker<Model>::~MonocularTracker() = default;

template class MonocularTracker<energy::model::PinholeCamera<Precision>>;
template class MonocularTracker<energy::model::SimpleRadialCamera<Precision>>;

}  // namespace dsopp::feature_based_slam::tracker
