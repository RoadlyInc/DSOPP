#ifndef DSOPP_FEATURE_BASED_SLAM_TRACKER_ADD_NEW_LAMDMARKS_HPP
#define DSOPP_FEATURE_BASED_SLAM_TRACKER_ADD_NEW_LAMDMARKS_HPP

#include <stddef.h>
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
/**
 * Add new landmarks to projections that do not match with other landmarks. It will be landmarks without position and
 * with one projection;
 *
 * @param landmarks object with all landmarks from the track
 * @param correspondences correspondences between new features and landmarks
 * @param feature_frame new features
 * @param frame_id id of the new frame
 */
void addNewLandmarks(track::Landmarks &landmarks, const std::vector<features::Correspondence> &correspondences,
                     const features::DistinctFeaturesFrame &feature_frame, size_t frame_id);

}  // namespace tracker
}  // namespace feature_based_slam
}  // namespace dsopp

#endif  // DSOPP_FEATURE_BASED_SLAM_TRACKER_ADD_NEW_LAMDMARKS_HPP
