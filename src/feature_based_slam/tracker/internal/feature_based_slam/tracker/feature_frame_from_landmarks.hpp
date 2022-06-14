#ifndef DSOPP_FEATURE_BASED_SLAM_TRACKER_CREATE_LANDMARKS_HPP
#define DSOPP_FEATURE_BASED_SLAM_TRACKER_CREATE_LANDMARKS_HPP

#include <stddef.h>
#include <deque>
#include <vector>

namespace dsopp {
namespace feature_based_slam {
namespace track {
struct Frame;
class Landmarks;
}  // namespace track
namespace tracker {
struct LandmarkFeatureFrame;
/**
 * Create 2d feature frame from the landmarks. This is required for use in matches. Now this function get all
 * projections of the landmarks on the frame with the given id.
 *
 * @param frames all frames from the track
 * @param landmarks all landmarks from the track
 * @param frame_id frame from which projections will be taken
 */
std::vector<LandmarkFeatureFrame> featureFrameFromLandmarks(std::deque<track::Frame> &frames,
                                                            track::Landmarks &landmarks, size_t frame_id);

}  // namespace tracker
}  // namespace feature_based_slam
}  // namespace dsopp

#endif  // DSOPP_FEATURE_BASED_SLAM_TRACKER_CREATE_LANDMARKS_HPP
