#ifndef DSOPP_SRC_ENERGY_PROBLEMS_COST_FUNCTORS_BUNDLE_ADJUSTMENT_GEOMETRIC_COST_FUNCTOR_HPP_
#define DSOPP_SRC_ENERGY_PROBLEMS_COST_FUNCTORS_BUNDLE_ADJUSTMENT_GEOMETRIC_COST_FUNCTOR_HPP_

#include <Eigen/Dense>

#include "common/settings.hpp"
#include "energy/motion/se3_motion.hpp"

namespace dsopp {
namespace energy {
namespace problem {
namespace cost_functors {

/**
 * \brief Function object which calculates reprojection error
 *
 * @tparam Model camera model type
 */
template <typename Model>
class BundleAdjustmentGeometricCostFunctor {
 public:
  /** number of residuals */
  static constexpr int residuals_num = 2;
  /**
   * constructor
   *
   * @param image_size image size
   * @param pixel coordinates of pixel observation
   * */
  BundleAdjustmentGeometricCostFunctor(const Eigen::Vector<Precision, 2> &image_size, const Eigen::Vector2d &pixel)
      : image_size_(image_size), pixel_(pixel) {}
  /**
   * Operator overloading for the ceres optimization
   *
   * @param t_c_w_ptr transformation from world to camera
   * @param point_3d_ptr landmark coordinates for the optimization
   * @param camera_intrinsics_ptr camera intrinsics vector
   * @param residuals_ptr cost to fill out
   * @return true
   * */
  template <class T>
  bool operator()(T const *const t_c_w_ptr, T const *const point_3d_ptr, T const *const camera_intrinsics_ptr,
                  T *residuals_ptr) const {
    Eigen::Map<Sophus::SE3<T> const> const t_c_w(t_c_w_ptr);
    Eigen::Map<Vec<T, 3> const> const point_3d(point_3d_ptr);
    Vec<T, Model::DoF> camera_intrinsics = Eigen::Map<const Vec<T, Model::DoF>>(camera_intrinsics_ptr);

    using ModelT = typename Model::template CastT<T>;
    ModelT model(image_size_, camera_intrinsics);

    Eigen::Vector<T, 2> pixel_projection;

    Eigen::Map<Vec<T, residuals_num>> residuals(residuals_ptr);
    if (model.template project(t_c_w * point_3d, pixel_projection)) {
      residuals = pixel_.template cast<T>() - pixel_projection;
    } else {
      residuals.setZero();
    }
    return true;
  }

 private:
  /** alias for Eigen vector from Scalar ``T`` ans length ``N`` */
  template <class T, int N>
  using Vec = Eigen::Vector<T, N>;
  /** image size */
  const Eigen::Vector<Precision, 2> image_size_;
  /** coordinates of pixel observation */
  const Eigen::Vector2d pixel_;
};
}  // namespace cost_functors
}  // namespace problem
}  // namespace energy
}  // namespace dsopp

#endif  // DSOPP_SRC_ENERGY_PROBLEMS_COST_FUNCTORS_BUNDLE_ADJUSTMENT_GEOMETRIC_COST_FUNCTOR_HPP_
