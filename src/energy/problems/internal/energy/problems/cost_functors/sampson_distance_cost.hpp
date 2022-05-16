#ifndef DSOPP_ENERGY_PROBLEMS_COST_FUNCTORS_SAMPSON_DISTANCE_COST_HPP_
#define DSOPP_ENERGY_PROBLEMS_COST_FUNCTORS_SAMPSON_DISTANCE_COST_HPP_

#include <Eigen/Dense>
#include <sophus/se3.hpp>

namespace dsopp::energy::problem::cost_functors {

/**
 * function to calculate sampson distance in pixels
 *
 * @param target target ray
 * @param E_t_r essential matrix from reference to target frame
 * @param inverse_focals inverse focals
 */
template <class T>
T sampsonDistance(const Eigen::Vector3<T> &target, const Eigen::Matrix<T, 3, 3> &E_t_r,
                  const Eigen::Vector3<T> &reference, T inverse_focal) {
  Eigen::Vector<T, 3> E_t_r_reference = E_t_r * reference;
  Eigen::Vector<T, 3> target_E_t_r = target.transpose() * E_t_r;

  T top = target.transpose() * E_t_r_reference;
  T bottom = (E_t_r_reference.template head<2>() * inverse_focal).squaredNorm() +
             (target_E_t_r.template head<2>() * inverse_focal).squaredNorm();

  if (bottom < T(1e-16)) return top;
  return top / sqrt(bottom);
}

/**
 * \brief const functor to evaluate sampson error for undistorted camera model
 *
 */
struct SampsonErrorCost {
  /**
   * constructor
   *
   * @param reference reference point
   * @param target target point
   */
  SampsonErrorCost(const Eigen::Vector2d &reference, const Eigen::Vector2d &target)
      : reference_(reference), target_(target) {}

  /**
   * @tparam T scalar
   * @param T_t_r_raw raw pointer to transfromation
   * @param focal_raw raw pointer to focal length
   * @param[out] res residuals
   * @return true if success
   */
  template <typename T>
  bool operator()(const T *const T_t_r_raw, const T *const focal_raw, T *res) const {
    using SE3 = Sophus::SE3<T>;
    using SO3 = Sophus::SO3<T>;

    Eigen::Map<const SE3> T_t_r(T_t_r_raw);
    T inverse_focal = T(1.) / *focal_raw;

    Eigen::Matrix<T, 3, 3> E_t_r = SO3::hat(T_t_r.translation()) * T_t_r.so3().matrix();

    const Eigen::Vector3<T> target_bearing = (target_.template cast<T>() * inverse_focal).homogeneous();
    const Eigen::Vector3<T> reference_bearing = (reference_.template cast<T>() * inverse_focal).homogeneous();

    res[0] = sampsonDistance<T>(target_bearing, E_t_r, reference_bearing, inverse_focal);
    return true;
  }

 private:
  /** reference ray */
  const Eigen::Vector2d reference_;
  /** target ray */
  const Eigen::Vector2d target_;
};

}  // namespace dsopp::energy::problem::cost_functors

#endif  // DSOPP_ENERGY_PROBLEMS_COST_FUNCTORS_SAMPSON_DISTANCE_COST_HPP_
