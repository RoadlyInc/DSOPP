#ifndef DSOPP_FEATURE_BASED_SLAM_TRACKER_ESTIMATE_SE3_PNP_HPP
#define DSOPP_FEATURE_BASED_SLAM_TRACKER_ESTIMATE_SE3_PNP_HPP

#include <deque>

#include "energy/motion/se3_motion.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"

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
 * estimate pose of frame based on solving PnP
 *
 * @tparam Calibration camera calibration type
 *
 * @param reference_frame_id, target_frame_id id's of the frames
 * @param correspondences_reference_to_target correspondences between features from the reference to the target frames.
 * @param target_features features from the frame that will be estimated
 * @param landmarks 3d landmarks to use PnP
 * @param[out] t_t_r estimated SE3 pose
 * @param camera_model camera model
 * @param pnp_ransac_threshold opengv style threshold for ransac
 * @return inlier size and all points used in PnP
 */
template <energy::model::Model Model>
std::pair<size_t, size_t> estimateSE3PnP(size_t frame_id, track::Landmarks &landmarks,
                                         energy::motion::SE3<Precision> &t_t_r, const Model &camera_model,
                                         const Precision pnp_ransac_threshold);
}  // namespace tracker
}  // namespace feature_based_slam
}  // namespace dsopp

#endif  // DSOPP_FEATURE_BASED_SLAM_TRACKER_ESTIMATE_SE3_PNP_HPP
