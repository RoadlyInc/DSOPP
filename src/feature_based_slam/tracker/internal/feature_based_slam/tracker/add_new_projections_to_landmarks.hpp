#ifndef DSOPP_FEATURE_BASED_SLAM_TRACKER_ADD_NEW_PROJECTIONS_TO_LANDMARKS_HPP
#define DSOPP_FEATURE_BASED_SLAM_TRACKER_ADD_NEW_PROJECTIONS_TO_LANDMARKS_HPP

#include <vector>

namespace dsopp {
namespace feature_based_slam {
namespace track {
class Landmarks;
}
namespace features {
struct Correspondence;
class DistinctFeaturesFrame;
}  // namespace features
namespace tracker {
struct LandmarkFeatureFrame;
/**
 * Add new projections from the new frame to the existing landmarks.
 *
 * @param landmarks object with all landmarks from the track
 * @param landmark_features landmark features that were used to find matches
 * @param correspondences correspondences between new features and landmarks
 * @param feature_frame new features
 * @param frame_id id of the new frame
 */
void addNewProjectionsToLandmarks(track::Landmarks &landmarks, const LandmarkFeatureFrame &landmark_features,
                                  const std::vector<features::Correspondence> &correspondences,
                                  const features::DistinctFeaturesFrame &feature_frame, size_t frame_id);

}  // namespace tracker
}  // namespace feature_based_slam
}  // namespace dsopp

#endif  // DSOPP_FEATURE_BASED_SLAM_TRACKER_ADD_NEW_PROJECTIONS_TO_LANDMARKS_HPP
