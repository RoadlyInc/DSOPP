#ifndef DSOPP_FEATURE_BASED_TRACKER_SLAM_ESTIMATE_SO3XS2_HPP
#define DSOPP_FEATURE_BASED_TRACKER_SLAM_ESTIMATE_SO3XS2_HPP

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
 * Estimate relative pose from target frame to reference frame
 *
 * @tparam Calibration camera calibration type
 *
 * @param reference_frame_points points on reference frame (modify found to inliers)
 * @param target_frame_points points on target frame (modify found to inliers)
 * @param[out] t_t_r transfromation from reference to target frame
 * matrix estimation)
 * @param number_of_threads number_of_threads
 * @param calibration camera calibration
 * @param min_ransac_matches minimum number of matches to perform ransac
 * @param essential_matrix_ransac_threshold opengv threshold for ransac
 * @return number of inliers and number of matching pairs
 */
template <energy::model::Model Model>
std::pair<size_t, size_t> estimateSO3xS2(size_t reference_frame_id, size_t target_frame_id, track::Landmarks &landmarks,
                                         energy::motion::SE3<Precision> &t_t_r, const size_t number_of_threads,
                                         const Model &camera_model, const size_t min_ransac_matches,
                                         const Precision essential_matrix_ransac_threshold,
                                         const Precision reprojection_threshold);
}  // namespace tracker
}  // namespace feature_based_slam
}  // namespace dsopp

#endif  // DSOPP_FEATURE_BASED_TRACKER_SLAM_ESTIMATE_SO3XS2_HPP
