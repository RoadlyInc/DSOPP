#ifndef DSOPP_FEATURE_BASED_SLAM_TRACK_ESTIMATE_SO3_INLIER_COUNT_HPP
#define DSOPP_FEATURE_BASED_SLAM_TRACK_ESTIMATE_SO3_INLIER_COUNT_HPP

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
 * Estimate number of inliers in rotation from target frame to reference frame
 *
 * @tparam Calibration camera calibration type
 * @tparam rotationNumberOfSamples number of samples in rotation ransace iteration
 *
 * @param reference_frame_points points on reference frame
 * @param target_frame_points points on target frame
 * @param rotation_t_r rotaiton from reference to target
 * @param camera_model camera model
 * @param rotation_ransac_iterations number of iterations in ransac
 * @param reprojection_threshold ransac threshold in pixels
 * @return number of inliers and number of matching pairs
 */
template <size_t rotationNumberOfSamples, energy::model::Model Model>
std::pair<size_t, size_t> estimateSO3inlierCount(size_t reference_frame_id, size_t target_frame_id,
                                                 track::Landmarks &landmarks, Sophus::SO3<Precision> &rotation_t_r,
                                                 const Model &camera_model, const size_t rotation_ransac_iterations,
                                                 const Precision reprojection_threshold);

}  // namespace tracker
}  // namespace feature_based_slam
}  // namespace dsopp

#endif  // DSOPP_FEATURE_BASED_SLAM_TRACK_ESTIMATE_SO3_INLIER_COUNT_HPP
