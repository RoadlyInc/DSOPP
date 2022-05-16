#ifndef DSOPP_SRC_ENERGY_PROBLEMS_LOCAL_PARAMETERIZATION_SE3_HPP_
#define DSOPP_SRC_ENERGY_PROBLEMS_LOCAL_PARAMETERIZATION_SE3_HPP_

#include <ceres/local_parameterization.h>
#include <sophus/se3.hpp>

namespace dsopp {
namespace energy {
namespace motion {
/** \brief Class for parameterization of the SE3 transformation
 *
 * In ceres SE3 transformation has 7dof : 4 for rotation and 3 for translation.
 * In fact SE3 has 6dof. This class is used to conversion to local parameters.
 */
class LocalParameterizationSE3 : public ceres::LocalParameterization {
 public:
  virtual ~LocalParameterizationSE3() {}

  /**
   * SE3 plus operation for Ceres
   * T * exp(x)
   *
   * @param T_raw transformation
   * @param delta_raw addition
   * @param[out] T_plus_delta_raw sum
   * @return true
   */
  virtual bool Plus(double const* T_raw, double const* delta_raw, double* T_plus_delta_raw) const {
    Eigen::Map<Sophus::SE3d const> const T(T_raw);
    Eigen::Map<Sophus::Vector6d const> const delta(delta_raw);
    Eigen::Map<Sophus::SE3d> T_plus_delta(T_plus_delta_raw);
    T_plus_delta = T * Sophus::SE3d::exp(delta);
    return true;
  }

  /**
   * Jacobian of SE3 plus operation for Ceres
   * Dx T * exp(x)  with  x=0
   *
   * @param T_raw transformation
   * @param[out] jacobian_raw jacobian
   * @return true
   */
  virtual bool ComputeJacobian(double const* T_raw, double* jacobian_raw) const {
    Eigen::Map<Sophus::SE3d const> T(T_raw);
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> jacobian(jacobian_raw);
    jacobian = T.Dx_this_mul_exp_x_at_0();
    return true;
  }
  /**
   * Number of global parameters
   *
   * @return number of global parameters
   * */
  virtual int GlobalSize() const { return Sophus::SE3d::num_parameters; }
  /**
   * Number of local parameters
   *
   * @return number of local parameters
   * */
  virtual int LocalSize() const { return Sophus::SE3d::DoF; }
};
}  // namespace motion
}  // namespace energy
}  // namespace dsopp

#endif  // DSOPP_SRC_ENERGY_PROBLEMS_LOCAL_PARAMETERIZATION_SE3_HPP_
