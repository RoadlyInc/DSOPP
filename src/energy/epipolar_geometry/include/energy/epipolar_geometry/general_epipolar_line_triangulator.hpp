#ifndef DSOPP_ENRGY_EPIPOLAR_GEOMETRY_GENERAL_EPIPOLAR_LINE_TRIANGULATOR_HPP_
#define DSOPP_ENRGY_EPIPOLAR_GEOMETRY_GENERAL_EPIPOLAR_LINE_TRIANGULATOR_HPP_

#include <Eigen/Dense>
#include "common/settings.hpp"
#include "energy/epipolar_geometry/triangulation.hpp"
#include "energy/motion/motion.hpp"

#include "energy/projector/camera_projector.hpp"

namespace dsopp::energy::epipolar_geometry {

/**
 * General inverse depth estimator a.k.a. `triangulator`
 *
 * estimates inverse depth on reference frame by known
 * inverse depth on target system and point on target image
 *
 * @tparam Model camera model type
 * @tparam Motion motion model type
 *
 */
template <class Model, motion::MotionProduct MotionProduct>
class EpipolarLineTriangulator {
 public:
  /**
   * @param model camera model
   * @param t_t_r transformation from reference to target frame
   * @param reference_point point on reference image
   */
  EpipolarLineTriangulator(const Model &model, const MotionProduct &t_t_r,
                           const Eigen::Vector2<Precision> &reference_point)
      : model_(model), t_t_r_(t_t_r), reference_point_(reference_point) {
    model_.unproject(reference_point_, reference_vec_);
    reference_vec_.normalize();
  }

  /**
   * estimates inverse depth in reference coord system
   *
   * @param point_target point on target image
   * @return inverse depth in reference coordinate system
   */
  Precision getInverseDepth(const Eigen::Vector2<Precision> &point_target) const {
    const Precision kDenominatorEps = 1e-8_p;
    Eigen::Matrix<Precision, 3, 4> t_r_t_matrix = t_t_r_.se3(reference_point_, point_target).inverse().matrix3x4();
    // TODO rewrite it in another LeastSqaures triangulation formulation
    Eigen::Vector3<Precision> vec_target;
    model_.unproject(point_target, vec_target);

    Eigen::Vector3<Precision> point_3d = triangulation(reference_vec_, vec_target.normalized(), t_r_t_matrix);
    Precision sign = point_3d(2) < 0.0_p ? -1 : 1;
    return 1.0_p / (point_3d(2) + sign * kDenominatorEps);
  }

 private:
  /** camera model */
  const Model &model_;
  /** transforamtion from reference to target frame */
  MotionProduct t_t_r_;
  /** bearing vector for reference point */
  Eigen::Vector3<Precision> reference_vec_;
  /** point on reference iamge */
  const Eigen::Vector2<Precision> reference_point_;
};
}  // namespace dsopp::energy::epipolar_geometry

#endif  // DSOPP_ENRGY_EPIPOLAR_GEOMETRY_GENERAL_EPIPOLAR_LINE_TRIANGULATOR_HPP_
