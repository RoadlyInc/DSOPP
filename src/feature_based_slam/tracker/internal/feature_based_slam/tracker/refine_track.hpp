#ifndef DSOPP_FEATURE_BASED_SLAM_TRACKER_OPTIMIZE_TRACK_HPP
#define DSOPP_FEATURE_BASED_SLAM_TRACKER_OPTIMIZE_TRACK_HPP

#include <deque>

#include "energy/motion/se3_motion.hpp"
#include "energy/problems/geometric_bundle_adjustment/ceres_geometric_bundle_adjustment.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"

namespace dsopp {
namespace feature_based_slam {
namespace track {
struct Frame;
struct Connections;
class Landmarks;
}  // namespace track
namespace tracker {

/**
 * traits to have non-const model type when it's optimized and const type otherwise
 */
template <bool T, energy::model::Model Model>
struct traits;
/**
 * non-const model type
 */
template <energy::model::Model Model>
struct traits<true, Model> {
  /** model type */
  using type = Eigen::Vector<Precision, Model::DoF>;
};
/**
 * const model type
 */
template <energy::model::Model Model>
struct traits<false, Model> {
  /** model type */
  using type = const Eigen::Vector<Precision, Model::DoF>;
};

/** shortcut for the fix parameters from the geometric bundle adjustment */
using ParameterParameterization = energy::problem::geometric_bundle_adjustment::ParameterParameterization;
/**
 * optimize all estimated transofrmations via reprojection error
 * @param landmarks 3d landmarks
 * @param number_of_threads number of threads
 * @param model camera model
 * @param[out] initialization_frames container with frames infromation
 * @param feature_matches LK flow feature correspondences
 * @param reprojection_threshold threshold in pixels for reproject error
 * @param last_frames_to_optimize number of the frames from the end of the track which we want to optimize
 * @param max_solver_time maximum time solver works in seconds
 * @param parameter_parameterization information about which parameters will be fixed during optimization
 */
template <energy::model::Model Model, bool OPTIMIZE_CAMERA = false>
void optimizeTransformations(track::Landmarks &landmarks, const size_t number_of_threads,
                             const Eigen::Vector<Precision, 2> &image_size,
                             typename traits<OPTIMIZE_CAMERA, Model>::type &camera_intrinsics,
                             std::deque<track::Frame> &initialization_frames, const Precision reprojection_threshold,
                             const size_t last_frames_to_optimize = 10, const Precision max_solver_time = 3._p,
                             ParameterParameterization parameter_parameterization = ParameterParameterization());

}  // namespace tracker
}  // namespace feature_based_slam
}  // namespace dsopp

#endif  // DSOPP_FEATURE_BASED_SLAM_TRACKER_OPTIMIZE_TRACK_HPP
