#ifndef DSOPP_ENERGY_MOTION_SE3_EPIPOLAR_LINE_TRIANGULATOR_HPP_
#define DSOPP_ENERGY_MOTION_SE3_EPIPOLAR_LINE_TRIANGULATOR_HPP_

#include <Eigen/Dense>
#include "common/settings.hpp"
#include "energy/motion/se3_motion.hpp"

namespace dsopp::energy::epipolar_geometry {
/**
 * Triangulator calculate idepth of the point using two projections
 */
class EpipolarLineTriangulatorSE3 {
 public:
  /**
   * Create triangulator for given point on the first frame
   *
   * @param t_r_t reference to target transformation
   * @param focal_lengths, principal_point intrinsic camera parameters
   * @param point_reference 2D point on the reference frame
   * @param max_idepth maximum idepth
   */
  EpipolarLineTriangulatorSE3(const energy::motion::SE3<Precision> &t_r_t,
                              const Eigen::Vector2<Precision> &focal_lengths,
                              const Eigen::Vector2<Precision> &principal_point,
                              const Eigen::Vector2<Precision> &point_reference, Precision max_idepth);
  /**
   * function for finding the inverse depth of the 3d point from two projections on two camera
   *
   * @param point_target projections on target camera positions
   * @return idepth inverse depth of the 3d point in the first camera
   */
  Precision getInverseDepth(const Eigen::Vector2<Precision> &point_target) const;

 private:
  /** maximum idepth */
  const Precision kMaxIdepth;
  /** mainimum idepth */
  const Precision kZeroIdepthEps = 1e-5_p;
  /** projection of the transformation vector */
  Eigen::Vector3<Precision> Kt_;
  /** reference bearing vector */
  Eigen::Vector3<Precision> bearing_vector_reference_;
  /** use X direction for triangulation */
  bool use_x_direction_ = true;
};

}  // namespace dsopp::energy::epipolar_geometry

#endif  // DSOPP_ENERGY_MOTION_SE3_EPIPOLAR_LINE_TRIANGULATOR_HPP_
