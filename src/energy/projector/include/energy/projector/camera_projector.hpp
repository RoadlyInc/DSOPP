#ifndef DSOPP_ENERGY_PROJECTOR_PROJECTOR_HPP_
#define DSOPP_ENERGY_PROJECTOR_PROJECTOR_HPP_

#include <optional>

#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"

namespace dsopp::energy::reprojection {

/**
 * \brief Projects `reference pixels` to `refernce-world`, `target-world` and points from
 * `target-world` to `target pixels`
 *
 * this class breaks t_target_refernce into 3 steps
 *
 * 1) `unproject`   -- from reference image and idepth to `reference-world` (camera coordinate system)
 * 2) `transform` -- from `reference-world` to `target-world`
 * 3) `project` -- from `target-world` to target image
 *
 * This object is needed to generalize above steps to general motion and camera models
 *
 * @tparam Scalar scalar type
 * @tparam Model camera model type
 * @tparam Motion motion model type
 */
template <class Scalar, class Model, energy::motion::MotionProduct MotionProduct>
class Projector;

/**
 * \brief projector specialization for SE3 motion class
 *
 * @tparam Scalar scalar type
 * @tparam Model camera model type
 */
template <class Scalar, class Model>
class Projector<Scalar, Model, energy::motion::SE3<Scalar>> {
 public:
  /** alias for R^2 vector */
  using Vector2 = Eigen::Vector<Scalar, 2>;
  /** alias for R^4 vector */
  using Vector4 = Eigen::Vector<Scalar, 4>;
  /**
   * @param model camera model
   * @param t_t_r se3 motion
   */
  Projector(const Model &model, const energy::motion::SE3<Scalar> &t_t_r) : model_(model), t_t_r_(t_t_r) {}

  /**
   * projects pixel and idepth to camera world
   *
   * @param pixel pixel coordiantes
   * @param idepth inverse depth
   * @param[out] point point in refernce camera world
   * @return true if success
   */
  bool unproject(const Vector2 &pixel, const Scalar idepth, Vector4 &point) const {
    Eigen::Vector<Scalar, 3> ray;
    bool success = model_.unproject(pixel, ray);

    point.template head<3>() = ray;
    point(3) = idepth;

    return success;
  }

  /**
   * projects point from `reference-world` to `target-world`
   *
   * @param point point in reference coordinate system
   * @return point int target coordiante system aka `target-world`
   */
  Vector4 transform(const Vector4 &point) const { return t_t_r_ * point; }

  /**
   * project point from `target-wrold` onto target image
   *
   * @param point point in target coordinate system
   * @param[out] pixel pixel on target image
   *@return true if success
   */
  bool project(const Vector4 &point, Vector2 &pixel) const {
    bool success;

    success = model_.project(point.template head<3>().eval(), pixel);
    return success && model_.validIdepth(point(3) / point(2));
  }

  /**
   * evaluate target cam inverse depth of projecting point
   * @param point projecting point
   * @return target cam inverse depth
   */
  Scalar evalTargetIdepth(const Vector4 &point, const Vector2 &) const { return point(3) / point(2); }

  /**
   * evaluate reference cam inverse depth of projecting point
   * @param point projecting point
   * @return reference cam inverse depth
   */
  Scalar evalReferenceIdepth(const Vector4 &point, const Vector2 &) const {
    Vector4 reference_point3d = t_t_r_.inverse() * point;
    return reference_point3d(3) / reference_point3d(2);
  }

  /**
   * project point from `target-wrold` onto target image
   *
   * @param point point in target coordinate system
   * @param[out] pixel pixel on target image
   * @param[out] jacobian jacobian of projection point onto image
   * @return true if success
   */
  bool project(const Vector4 &point, Vector2 &pixel, Eigen::Matrix<Scalar, 2, 4> &jacobian) const {
    bool success;
    Eigen::Matrix<Scalar, 2, 3> project_jacobian;
    success = model_.project(point.template head<3>().eval(), pixel, project_jacobian);
    success &= model_.validIdepth(point(3) / point(2));

    jacobian.template block<2, 3>(0, 0) = project_jacobian;
    jacobian.col(3).setZero();
    return success;
  }

 private:
  /** camera model */
  const Model &model_;
  /** transformation from reference to target */
  const energy::motion::SE3<Scalar> t_t_r_;
};

}  // namespace dsopp::energy::reprojection

#endif  // DSOPP_ENERGY_PROJECTOR_PROJECTOR_HPP_
