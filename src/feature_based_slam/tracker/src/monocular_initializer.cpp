#include "feature_based_slam/tracker/monocular_initializer.hpp"

#include <glog/logging.h>

#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/camera_model/pinhole/simple_radial.hpp"
#include "feature_based_slam/features/correspondences_finder.hpp"
#include "feature_based_slam/features/distinct_feature.hpp"
#include "feature_based_slam/features/distinct_features_extractor.hpp"
#include "feature_based_slam/features/distinct_features_frame.hpp"
#include "feature_based_slam/features/optical_flow.hpp"
#include "feature_based_slam/features/orb.hpp"
#include "feature_based_slam/initialization_strategy/initializer_keyframe_strategy.hpp"
#include "feature_based_slam/tracker/add_new_landmarks.hpp"
#include "feature_based_slam/tracker/add_new_projections_to_landmarks.hpp"
#include "feature_based_slam/tracker/estimate_so3_inlier_count.hpp"
#include "feature_based_slam/tracker/feature_frame_from_landmarks.hpp"
#include "feature_based_slam/tracker/initialize_poses.hpp"
#include "feature_based_slam/tracker/landmark_feature_frame.hpp"
#include "features/camera/camera_features.hpp"
#include "features/camera/pixel_data_frame.hpp"
#include "sensors/camera_calibration/mask/camera_mask.hpp"

namespace dsopp::feature_based_slam::tracker {

template <energy::model::Model Model>
MonocularInitializer<Model>::MonocularInitializer(
    track::Track &track, const size_t sensor_id, const features::DistinctFeaturesExtractor &feature_extractor,
    std::unique_ptr<initialization_strategy::InitializerKeyframeStrategy> &&initializer_keyframe_strategy,
    const Options &options, std::unique_ptr<Model> &&model)
    : Initializer(track, sensor_id, feature_extractor, options),
      model_(std::move(model)),
      initializer_keyframe_strategy_(std::move(initializer_keyframe_strategy)) {}

template <energy::model::Model Model>
void MonocularInitializer<Model>::tick(const sensors::SynchronizedFrame &frame_data, const size_t number_of_threads) {
  static constexpr size_t kRotationNumberOfSamples = 3;

  auto &camera_features = frame_data.cameraFeatures();
  if (!camera_features.contains(sensor_id_)) {
    return;
  }

  auto &frame = *camera_features.at(sensor_id_);
  const auto &mask = frame.pyramidOfMasks()[0];

  auto feature_frames = featureFrameFromLandmarks(track_.frames, track_.landmarks,
                                                  track_.frames.empty() ? 0 : track_.frames.front().frame_id);
  auto [features, correspondences_from_to] =
      features::findCorrespondences(feature_frames[0].features.get(), feature_frames[0].image, frame.frameData(), mask,
                                    feature_extractor_, features::OpticalFlowMatch);
  addNewProjectionsToLandmarks(track_.landmarks, feature_frames[0], correspondences_from_to, *features, frame.id());
  addNewLandmarks(track_.landmarks, correspondences_from_to, *features, frame.id());

  if (track_.frames.empty()) {
    track_.frames.emplace_back(frame.timestamp(), frame.id(), frame.frameData());
    return;
  }

  size_t found_points = correspondences_from_to.size();
  size_t points = features->features().size();

  const Precision kMinMatchesRatio = 0.4_p;
  const size_t matches_ratio = static_cast<size_t>(kMinMatchesRatio * static_cast<Precision>(points));
  VLOG(1) << "Matches : " << found_points << " (" << points << ")";
  if (found_points < std::max(this->options_.min_ransac_matches, matches_ratio)) {
    clear();
    return;
  }

  size_t so3_inlier_number = 0, s_matches_number = correspondences_from_to.size();

  Sophus::SO3<Precision> rotation_t_r;

  if (initializer_keyframe_strategy_->managesStandstill()) {
    auto [inlier_number, matching_number] = estimateSO3inlierCount<kRotationNumberOfSamples>(
        track_.frames.back().frame_id, frame.id(), track_.landmarks, rotation_t_r, *model_,
        this->options_.rotation_ransac_iterations, this->options_.reprojection_threshold);
    so3_inlier_number = inlier_number;
    s_matches_number = matching_number;
  }

  auto frame_status =
      initializer_keyframe_strategy_->needNewFrame(frame, track_.frames, so3_inlier_number, s_matches_number);

  if (frame_status == initialization_strategy::FrameStatus::kDropFrame) {
    /** NOTE here we would not estimate these images poses via pnp in the future
     * becuase stops could take about some thousands of frames and it would not be
     * even near realtime.
     */
    track_.frames.back().attached_frames.push_back(
        track::Frame::AttachedFrame{frame.timestamp(), frame.id(), rotation_t_r});
    return;
  }

  track_.frames.emplace_back(frame.timestamp(), frame.id(), frame.frameData());

  if (frame_status == initialization_strategy::FrameStatus::kFinish) {
    is_initialized_ = initializePoses<Model>(track_, *model_, this->options_, number_of_threads);
    if (!is_initialized_) clear();
  }
}

template <energy::model::Model Model>
Eigen::VectorX<Precision> MonocularInitializer<Model>::cameraIntrinsics() const {
  return model_->intrinsicsParameters();
}

template <energy::model::Model Model>
MonocularInitializer<Model>::~MonocularInitializer() = default;

template class MonocularInitializer<energy::model::PinholeCamera<Precision>>;
template class MonocularInitializer<energy::model::SimpleRadialCamera<Precision>>;

}  // namespace dsopp::feature_based_slam::tracker
