#include "feature_based_slam/tracker/add_new_projections_to_landmarks.hpp"

#include "feature_based_slam/features/correspondence.hpp"
#include "feature_based_slam/track/landmarks.hpp"
#include "feature_based_slam/tracker/landmark_feature_frame.hpp"

namespace dsopp {
namespace feature_based_slam {
namespace tracker {
void addNewProjectionsToLandmarks(track::Landmarks &landmarks, const LandmarkFeatureFrame &landmark_features,
                                  const std::vector<features::Correspondence> &correspondences,
                                  const features::DistinctFeaturesFrame &feature_frame, size_t frame_id) {
  const auto &features = feature_frame.features();
  for (const auto &correspondence : correspondences) {
    if (correspondence.is_inlier) {
      auto landmark_idx = landmark_features.landmark_idx[correspondence.idx_from];
      landmarks.addProjectionToLandmark(landmark_idx, frame_id, features[correspondence.idx_to]);
    }
  }
}

}  // namespace tracker
}  // namespace feature_based_slam
}  // namespace dsopp
