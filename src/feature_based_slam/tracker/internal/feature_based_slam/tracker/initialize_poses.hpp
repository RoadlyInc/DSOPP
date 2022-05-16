#ifndef DSOPP_FEATURE_BASED_TRACKER_SLAM_INITIALIZE_POSES_HPP
#define DSOPP_FEATURE_BASED_TRACKER_SLAM_INITIALIZE_POSES_HPP

#include "energy/motion/se3_motion.hpp"
#include "feature_based_slam/tracker/initializer.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"

namespace dsopp {
namespace feature_based_slam {
namespace track {
struct Track;
struct Connections;
}  // namespace track
namespace tracker {
/**
 * Initialize feature based slam tracker
 *
 * @tparam Calibration camera calibration type
 *
 * @param track track with the frames and landmarks
 * @param connections correspondences between 2d features in the frames
 * @param model camera model
 * @param options all constants which will be used in the feature based slam algorithms and optimizations
 * @return true if tracker was initialized and false otherwise
 */
template <energy::model::Model Model>
bool initializePoses(track::Track &track, const Model &model, const Options &options, const size_t number_of_threads);

}  // namespace tracker
}  // namespace feature_based_slam
}  // namespace dsopp
#endif  // DSOPP_FEATURE_BASED_SLAM_TRACKER_INITIALIZE_POSES_HPP
