#include "feature_based_slam/tracker/add_new_landmarks.hpp"

#include "feature_based_slam/features/correspondence.hpp"
#include "feature_based_slam/track/landmarks.hpp"
#include "feature_based_slam/tracker/landmark_feature_frame.hpp"

namespace dsopp {
namespace feature_based_slam {
namespace tracker {
void addNewLandmarks(track::Landmarks &landmarks, const std::vector<features::Correspondence> &correspondences,
                     const features::DistinctFeaturesFrame &feature_frame, size_t frame_id) {
  const auto &features = feature_frame.features();
  std::vector<bool> look_up(features.size(), false);
  for (const auto &correspondence : correspondences) {
    if (correspondence.is_inlier) {
      look_up[correspondence.idx_to] = true;
    }
  }

  for (size_t i = 0; i < features.size(); ++i) {
    if (!look_up[i]) {
      auto landmark_idx = landmarks.add();
      landmarks.addProjectionToLandmark(landmark_idx, frame_id, features[i]);
    }
  }
}

}  // namespace tracker
}  // namespace feature_based_slam
}  // namespace dsopp
