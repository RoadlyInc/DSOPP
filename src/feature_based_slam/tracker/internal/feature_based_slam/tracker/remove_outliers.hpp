#ifndef DSOPP_FEATURE_BASED_SLAM_TRACKER_REMOVE_OUTLIERS_HPP
#define DSOPP_FEATURE_BASED_SLAM_TRACKER_REMOVE_OUTLIERS_HPP

#include <deque>

#include "energy/motion/se3_motion.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"

namespace dsopp {
namespace feature_based_slam {
namespace track {
struct Frame;
class Landmarks;
}  // namespace track
namespace tracker {

/**
 * remove projections of landmarks that are very different from the corresponding features of the frame
 *
 * @param landmarks 3d landmarks
 * @param model camera model
 * @param frames container with frames information
 * @param last_frames_to_filter number of the frames from the end of the track which we want to clear
 * @param outlier_threshold outlier threshold in pixels
 */
template <energy::model::Model Model>
void removeOutliers(track::Landmarks &landmarks, const Model &model, std::deque<track::Frame> &frames,
                    const size_t last_frames_to_filter = 10, const Precision outlier_threshold = 0.5);

}  // namespace tracker
}  // namespace feature_based_slam
}  // namespace dsopp

#endif  // DSOPP_FEATURE_BASED_SLAM_TRACKER_REMOVE_OUTLIERS_HPP
