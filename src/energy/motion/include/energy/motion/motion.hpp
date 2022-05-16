#ifndef DSOPP_SRC_ENERGY_MOTION_MOTION_HPP_
#define DSOPP_SRC_ENERGY_MOTION_MOTION_HPP_

#include <Eigen/Dense>

namespace dsopp::energy::motion {

/**
 * \brief concept for motion type
 */
template <class Transformation>
concept Motion = requires(Transformation t, const Eigen::Vector<typename Transformation::Scalar, 3> &pt3d) {
  { t *(t.inverse() * t) }
  ->std::same_as<Transformation>;
  { t.inverse() }
  ->std::same_as<typename Transformation::Inverse>;
  { t.inverse() * t }
  ->std::same_as<typename Transformation::Product>;
  { Transformation::DoF }
  ->std::convertible_to<int>;
  { Transformation::num_parameters }
  ->std::convertible_to<int>;
  { t *pt3d }
  ->std::same_as<Eigen::Vector<typename Transformation::Scalar, 3>>;
  { t *Eigen::Vector<typename Transformation::Scalar, 4>(pt3d.homogeneous()) }
  ->std::same_as<Eigen::Vector<typename Transformation::Scalar, 4>>;
};

/**
 * \brief concept for motion product type
 */
template <class Transformation>
concept MotionProduct = requires(Transformation t, const Eigen::Vector<typename Transformation::Scalar, 3> &pt3d,
                                 typename Transformation::Motion t_w_a) {
  { Transformation::DoF }
  ->std::convertible_to<int>;
  { Transformation::num_parameters }
  ->std::convertible_to<int>;
  { t *pt3d }
  ->std::same_as<Eigen::Vector<typename Transformation::Scalar, 3>>;
  { t *Eigen::Vector<typename Transformation::Scalar, 4>(pt3d.homogeneous()) }
  ->std::same_as<Eigen::Vector<typename Transformation::Scalar, 4>>;
  { t.Adj() }
  ->std::same_as<Eigen::Matrix<typename Transformation::Scalar, Transformation::DoF, Transformation::DoF>>;
  { t_w_a *t }
  ->std::same_as<typename Transformation::Motion>;
};

}  // namespace dsopp::energy::motion

#endif  // DSOPP_SRC_ENERGY_MOTION_MOTION_HPP_
