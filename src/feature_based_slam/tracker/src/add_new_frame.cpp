#include "feature_based_slam/tracker/add_new_frame.hpp"

#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/camera_model/pinhole/simple_radial.hpp"
#include "feature_based_slam/features/correspondences_finder.hpp"
#include "feature_based_slam/features/optical_flow.hpp"
#include "feature_based_slam/track/frame.hpp"
#include "feature_based_slam/track/track.hpp"
#include "feature_based_slam/tracker/add_new_landmarks.hpp"
#include "feature_based_slam/tracker/add_new_projections_to_landmarks.hpp"
#include "feature_based_slam/tracker/estimate_so3_inlier_count.hpp"
#include "feature_based_slam/tracker/feature_frame_from_landmarks.hpp"
#include "feature_based_slam/tracker/landmark_feature_frame.hpp"
#include "features/camera/camera_features.hpp"
#include "sensors/camera_calibration/mask/camera_mask.hpp"

namespace dsopp::feature_based_slam::tracker {

template <energy::model::Model Model>
void addNewFrame(track::Track &track, dsopp::features::CameraFeatures &frame, const Model &model,
                 const features::DistinctFeaturesExtractor &feature_extractor, Precision reprojection_threshold,
                 size_t rotation_ransac_iterations) {
  const Precision kMaxRatioOfSo3Inliers = 0.7_p;
  static constexpr size_t kRotationNumberOfSamples = 3;

  const auto &mask = frame.pyramidOfMasks()[0];

  // extract features and connect with landmark projections from the LAST frame
  auto feature_frames = featureFrameFromLandmarks(track.frames, track.landmarks, track.frames.back().frame_id);
  auto [features, correspondences_from_to] =
      features::findCorrespondences(feature_frames[0].features.get(), feature_frames[0].image, frame.frameData(), mask,
                                    feature_extractor, features::OpticalFlowMatch);
  addNewProjectionsToLandmarks(track.landmarks, feature_frames[0], correspondences_from_to, *features, frame.id());
  addNewLandmarks(track.landmarks, correspondences_from_to, *features, frame.id());

  // check standstill
  Sophus::SO3<Precision> rotation_t_r;
  auto [so3_inlier_number, so3_matching_number] = estimateSO3inlierCount<kRotationNumberOfSamples>(
      track.frames.back().frame_id, frame.id(), track.landmarks, rotation_t_r, model, rotation_ransac_iterations,
      reprojection_threshold);
  if (Precision(so3_inlier_number) / Precision(so3_matching_number) > kMaxRatioOfSo3Inliers) {
    track.frames.back().attached_frames.push_back(
        track::Frame::AttachedFrame{frame.timestamp(), frame.id(), rotation_t_r});
    return;
  }
  track.frames.emplace_back(frame.timestamp(), frame.id(), frame.frameData());
}

template void addNewFrame<energy::model::PinholeCamera<Precision>>(track::Track &, dsopp::features::CameraFeatures &,
                                                                   const energy::model::PinholeCamera<Precision> &,
                                                                   const features::DistinctFeaturesExtractor &,
                                                                   Precision, size_t);

template void addNewFrame<energy::model::SimpleRadialCamera<Precision>>(
    track::Track &, dsopp::features::CameraFeatures &, const energy::model::SimpleRadialCamera<Precision> &,
    const features::DistinctFeaturesExtractor &, Precision, size_t);

}  // namespace dsopp::feature_based_slam::tracker
