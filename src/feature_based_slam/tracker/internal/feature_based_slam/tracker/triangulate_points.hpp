#ifndef DSOPP_FEATURE_BASED_SLAM_TRACKER_TRIANGULATE_POINTS_HPP
#define DSOPP_FEATURE_BASED_SLAM_TRACKER_TRIANGULATE_POINTS_HPP

#include <deque>

#include "energy/motion/se3_motion.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"

namespace dsopp {
namespace feature_based_slam {
namespace track {
struct Frame;
class Landmarks;
}  // namespace track
namespace features {
struct Correspondence;
class DistinctFeaturesFrame;
}  // namespace features
namespace tracker {

/**
 * triangulate points between two given frames
 *
 * @tparam Model camera model type
 *
 * @param reference_frame_id, target_frame_id id's of the frames
 * @param[out] landmarks new landmarks will be stored here
 * @param t_w_r, t_w_t positions of reference and target frames
 * @param camera_model camera model
 * @param triangulation_cos_threshold triangulation threshold for bearing vector cos
 */
template <energy::model::Model Model>
void triangulatePoints(size_t reference_frame_id, size_t target_frame_id, track::Landmarks &landmarks,
                       const energy::motion::SE3<Precision> &t_w_r, const energy::motion::SE3<Precision> &t_w_t,
                       const Model &camera_model, const Precision triangulation_cos_threshold);

/**
 * triangulate all uninitialized landmarks using all available projections
 *
 * @tparam Model camera model type
 *
 * @param[out] landmarks new landmarks will be stored here
 * @param frames all frames from the track
 * @param camera_model camera model
 * @param min_number_of_projections only landmarks with min number of projections triangulated
 * @param retriangulation if true, all point will be triangulated, even they have positions
 */
template <energy::model::Model Model>
void triangulatePoints(track::Landmarks &landmarks, std::deque<track::Frame> &frames, const Model &camera_model,
                       size_t min_number_of_projections, bool retriangulation = false);

}  // namespace tracker
}  // namespace feature_based_slam
}  // namespace dsopp

#endif  // DSOPP_FEATURE_BASED_SLAM_TRACKER_TRIANGULATE_POINTS_HPP
