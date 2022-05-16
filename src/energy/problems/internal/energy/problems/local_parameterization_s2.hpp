#ifndef DSOPP_SRC_ENERGY_PROBLEMS_LOCAL_PARAMETERIZATION_S2_HPP_
#define DSOPP_SRC_ENERGY_PROBLEMS_LOCAL_PARAMETERIZATION_S2_HPP_

#include <ceres/local_parameterization.h>

namespace dsopp {
namespace energy {
namespace problem {
/** \brief Class for parameterization of the S2 unit sphere
 *
 * Vector has 3dof : spatial coordinates.
 * In fact S2 has 2dof. This class is used to conversion to local parameters.
 */
class LocalParameterizationS2 : public ceres::LocalParameterization {
 public:
  virtual ~LocalParameterizationS2() {}

  /**
   * S2 plus operation for Ceres
   *
   * @param vector_raw vector
   * @param delta_raw addition
   * @param[out] vector_plus_delta_raw sum
   * @return true
   */
  virtual bool Plus(double const* vector_raw, double const* delta_raw, double* vector_plus_delta_raw) const {
    Eigen::Vector2d vector_local_parameterization;
    // clang-format off
    vector_local_parameterization << acos(vector_raw[2]),
                                     atan2(vector_raw[1], vector_raw[0]);
    // clang-format on
    Eigen::Map<Eigen::Vector2d const> const delta(delta_raw);
    vector_local_parameterization += delta;
    const double theta = vector_local_parameterization[0];
    const double phi = vector_local_parameterization[1];
    Eigen::Map<Eigen::Vector3d> vector_plus_delta(vector_plus_delta_raw);
    // clang-format off
    vector_plus_delta << sin(theta) * cos(phi),
                         sin(theta) * sin(phi),
                         cos(theta);
    // clang-format on
    return true;
  }

  /**
   * Jacobian of S2 plus operation for Ceres
   *
   * @param vector_raw vector
   * @param[out] jacobian_raw jacobian
   * @return true
   */
  virtual bool ComputeJacobian(double const* vector_raw, double* jacobian_raw) const {
    const double theta = acos(vector_raw[2]);
    const double phi = atan2(vector_raw[1], vector_raw[0]);
    Eigen::Map<Eigen::Matrix<double, 3, 2, Eigen::RowMajor>> jacobian(jacobian_raw);
    // clang-format off
    jacobian << cos(theta) * cos(phi), sin(theta) * (-sin(phi)),
                cos(theta) * sin(phi), sin(theta) * cos(phi),
                -sin(theta), 0;
    // clang-format on
    return true;
  }
  /**
   * Number of global parameters
   *
   * @return number of global parameters
   * */
  virtual int GlobalSize() const { return 3; }
  /**
   * Number of local parameters
   *
   * @return number of local parameters
   * */
  virtual int LocalSize() const { return 2; }
};
}  // namespace problem
}  // namespace energy
}  // namespace dsopp

#endif  // DSOPP_SRC_ENERGY_PROBLEMS_LOCAL_PARAMETERIZATION_S2_HPP_
