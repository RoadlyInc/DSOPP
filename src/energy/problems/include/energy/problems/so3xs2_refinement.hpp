#ifndef DSOPP_ENERGY_PROBLEM_SO3XS2_REFINEMENT_HPP_
#define DSOPP_ENERGY_PROBLEM_SO3XS2_REFINEMENT_HPP_

#include "energy/camera_model/pinhole/pinhole_camera.hpp"

#include <ceres/ceres.h>
#include <sophus/se3.hpp>

namespace dsopp::energy::problem {

/**
 * method to optimize pose as essential matrix
 *
 * @param focal focal lengths of camera
 * @param[out] t_t_r transformation from reference to target
 * @param points_reference projections from reference frame
 * @param points_target projections from target frame
 * @param threshold huber-loss threshold in pixels
 * @param number_of_threads
 */
template <bool OPTIMIZE_FOCAL = false, bool OPTIMIZE_POSE = true>
void refineSO3xS2(typename std::conditional<OPTIMIZE_FOCAL, double &, double>::type focal, Sophus::SE3d &t_t_r,
                  const std::vector<Eigen::Vector2d> &points_reference,
                  const std::vector<Eigen::Vector2d> &points_target, double threshold = 1.5,
                  const int number_of_threads = 1);

}  // namespace dsopp::energy::problem

#endif  // DSOPP_ENERGY_PROBLEM_SO3XS2_REFINEMENT_HPP_
