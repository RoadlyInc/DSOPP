#ifndef DSOPP_SRC_ENERGY_MOTION_SE3_MOTION_HPP_
#define DSOPP_SRC_ENERGY_MOTION_SE3_MOTION_HPP_

#include <Eigen/Dense>
#include <sophus/se3.hpp>

namespace dsopp::energy::motion {

class EpipolarLineTriangulatorSE3;
/**
 * \brief class for representing SE3
 *
 * @tparam Scalar scalar type
 */
template <class Scalar_ = double>
class SE3 : public Sophus::SE3<Scalar_> {
 public:
  /** alias for Scalar type */
  using Scalar = Scalar_;
  /** alias for number of parameters */
  using Sophus::SE3<Scalar_>::num_parameters;
  /** alias for Motion Product Type (SE3 coincidence with its Product)*/
  using Product = SE3;
  /** alias for Motion type of Product type (SE3 coincidence with its Product)*/
  using Motion = SE3;
  /** alias for Motion Inverse Type */
  using Inverse = SE3;
  /** type of a casted motion */
  template <typename T>
  using CastT = SE3<T>;
  /** constructor from Sophus
   * @param se3 input se3
   * */
  SE3(const Sophus::SE3<Scalar>& se3 = Sophus::SE3<Scalar>()) : Sophus::SE3<Scalar>(se3) {}
  /** constructor from another sophus se3
   * @tparam SE3Derived derived type for SE3Base
   * @param rhs se3 input*/
  template <class SE3Derived>
  SE3(const Sophus::SE3Base<SE3Derived>& rhs) : Sophus::SE3<Scalar>(rhs) {}

  /**
   * constructor from rotation matrix and translation
   * @param R rotation matrix
   * @param T translation
   */
  SE3(const Eigen::Matrix<Scalar, 3, 3>& R, const Eigen::Vector<Scalar, 3>& T) : SE3(Sophus::SE3<Scalar>(R, T)) {}

  /**
   * constructor from quaternion and translation
   * @param Q quaternion
   * @param T translation
   */
  SE3(const Eigen::Quaternion<Scalar>& Q, const Eigen::Vector<Scalar, 3>& T) : SE3(Sophus::SE3<Scalar>(Q, T)) {}

  /** inverse transformation
   * @return invertd transformation
   */
  SE3 inverse() const { return SE3(Sophus::SE3<Scalar>::inverse()); }
  /** alias for all operator* from sophus */
  using Sophus::SE3<Scalar>::operator*;

  /** get se3 object
   *
   * @return se3 object
   * */
  const SE3& se3() const { return *this; }

  /** get se3 object by points on reference and target frame
   *
   * @return se3 object
   * */
  const SE3& se3(const Eigen::Vector2<Scalar>&, const Eigen::Vector2<Scalar>&) const { return *this; }

  /** operator* for SE3 class
   * @param rhs right hand side of multiplication
   * @return multiplication result
   */
  SE3 operator*(const SE3<Scalar>& rhs) const {
    return SE3(Sophus::SE3<Scalar>::operator*(static_cast<Sophus::SE3<Scalar>>(rhs)));
  }

  /**
   * see Sophus::SE3<Scalar>::exp for detials
   * @param log map of object
   * @return object
   */
  static SE3 exp(const Eigen::Vector<Scalar, Sophus::SE3<Scalar>::DoF> log) {
    return SE3(Sophus::SE3<Scalar>::exp(log));
  }
  /**
   * cast SE3 to another scalar
   * @tparam ScalarNew new scalar type
   * @return casted SE3
   */
  template <class ScalarNew>
  SE3<ScalarNew> cast() const {
    return SE3<ScalarNew>(Sophus::SE3<Scalar>::template cast<ScalarNew>());
  }

  /**
   * Jacobian of group action with left incerementation:
   * d ((e^`eps` this) [x | idepth])
   *       d`eps`
   *
   * @param p 3d point
   * @param idepth inverse depth
   * @return group action Jacobian of lie algebra parameterization
   */
  Eigen::Matrix<Scalar, 3, 6> leftBoxPlusJacobian(const Eigen::Vector<Scalar, 3>& p,
                                                  const Scalar& idepth = Scalar(1)) const {
    Eigen::Matrix<Scalar, 3, 6> J;
    J.template block<3, 3>(0, 0) = idepth * Eigen::Matrix<Scalar, 3, 3>::Identity();
    J.template block<3, 3>(0, 3) = -Sophus::SO3<Scalar>::hat(this->transform(p, idepth));
    return J;
  }

  /**
   * Jacobian of group action with right incerementation:
   * d ( (this e^`eps`) [x | idepth])
   *       d`eps`
   *
   * @param p 3d point
   * @param idepth inverse depth
   * @return group action Jacobian of lie algebra parameterization
   */
  Eigen::Matrix<Scalar, 3, 6> rightBoxPlusJacobian(const Eigen::Vector<Scalar, 3>& p,
                                                   const Scalar& idepth = Scalar(1)) const {
    auto matrix = this->rotationMatrix();
    Eigen::Matrix<Scalar, 3, 6> J;
    J.template block<3, 3>(0, 0) = idepth * matrix;
    J.template block<3, 3>(0, 3) = -matrix * Sophus::SO3<Scalar>::hat(p);
    return J;
  }
  /**
   * Calculation of the relative transformation uncertainty
   *
   * We have
   * T_w_1 = T_w_1_zero * e^`eps_1` - estimated first pose
   * T_w_2 = T_w_2_zero * e^`eps_2` - estimated second pose
   * sigma_11, sigma_22, sigma_12 - covariance matrices between eps_1 and eps_2
   *
   * Relative pose
   * T_1_2 = T_1_2_zero * e^`eps` = (T_w_1^`-1` * T_w_2)
   *
   * And we want to have covariance matrix of the eps
   *
   * @param t_w_1, t_w_2 first and second poses
   * @param sigma_11, sigma_12, sigma_22 covariance matrices between eps_1 and eps_2
   * @return uncertainty of the relative transformation
   */
  static Eigen::Matrix<Scalar, 6, 6> relativeTransformationUncertainty(const SE3<Scalar>& t_w_1,
                                                                       const SE3<Scalar>& t_w_2,
                                                                       const Eigen::Matrix<Scalar, 6, 6>& sigma_11,
                                                                       const Eigen::Matrix<Scalar, 6, 6>& sigma_22,
                                                                       const Eigen::Matrix<Scalar, 6, 6>& sigma_12) {
    Eigen::Matrix<Scalar, 6, 6> adj = (t_w_2.inverse() * t_w_1).Adj();
    return adj * sigma_11 * adj.transpose() - sigma_12.transpose() * adj.transpose() - adj * sigma_12 + sigma_22;
  }

  /**
   * transformation for point adn idepth
   *
   * @param p point
   * @param idepth inverse depth
   * @return transformed point
   */
  Eigen::Vector<Scalar, 3> transform(const Eigen::Vector<Scalar, 3>& p, const Scalar& idepth) const {
    return this->so3() * p + idepth * this->translation();
  }

  /**
   * Group action Jacobian with respect to quaternion and translation
   *
   * d (imag(q [0| x] q^*) + id * t )
   * d q, t
   *
   * @param p rhs point of group action
   * @param idepth inverse depth
   * @return Jacobian
   */
  Eigen::Matrix<Scalar, 3, 7> actionJacobianQuaternionTranslation(const Eigen::Vector<Scalar, 3>& p,
                                                                  const Scalar& idepth = Scalar(1)) const {
    Eigen::Matrix<Scalar, 3, 7> J_quaternion_translation;
    J_quaternion_translation.template block<3, 3>(0, 4) = idepth * Eigen::Matrix<Scalar, 3, 3>::Identity();

    Eigen::Vector<Scalar, 4> q = this->unit_quaternion().coeffs();
    Eigen::Matrix<Scalar, 3, 3> qv_hat = Sophus::SO3<Scalar>::hat(q.template head<3>());

    J_quaternion_translation.col(3) = Scalar(2) * qv_hat * p;

    Eigen::Matrix<Scalar, 3, 3> uv_hat = Sophus::SO3<Scalar>::hat(J_quaternion_translation.col(3));
    Eigen::Matrix<Scalar, 3, 3> p_hat = Sophus::SO3<Scalar>::hat(p);

    J_quaternion_translation.template block<3, 3>(0, 0) = -Scalar(2) * (q(3) * p_hat + qv_hat * p_hat) - uv_hat;

    return J_quaternion_translation;
  }

  /**
   * model internal parameters
   *
   * @return parameters
   */
  Eigen::Vector<Scalar, num_parameters> parameters() const {
    Eigen::Vector<Scalar, num_parameters> params;
    params = Eigen::Map<const Eigen::Vector<Scalar, num_parameters>>(this->data());
    return params;
  }

  /**
   * set rotation matrix to `R`
   *
   * @param R new rotation matrix
   */
  void setRotationMatrix(const Eigen::Matrix<Scalar, 3, 3>& R) { this->so3() = Sophus::SO3<Scalar>::fitToSO3(R); }
  /**
   * model internal parameters setter
   *
   * @param params new internal parameters
   */
  void setParameters(const Eigen::Vector<Scalar, num_parameters>& params) {
    Eigen::Map<Eigen::Vector<Scalar, num_parameters>>(this->data()) = params;
  }

  /**
   * increment current state on the left by lifting `eps` to the exponent
   *
   * @param eps increment
   * @return incremented object
   */
  SE3 leftIncrement(const Eigen::Vector<Scalar, 6>& eps) const { return this->exp(eps) * (*this); }

  /**
   * increment current state on the right by lifting `eps` to the exponent
   *
   * @param eps increment
   * @return incremented object
   */
  SE3 rightIncrement(const Eigen::Vector<Scalar, 6>& eps) const { return (*this) * this->exp(eps); }

  /**
   * Linear operator of transforming log to right log part
   * @return se3 Adjoint
   */
  Eigen::Matrix<Scalar, 6, 6> rightLogTransformer() const { return this->Adj(); }
  /**
   * Linear operator of transforming log to left log part
   *
   * In case of SE3 this is just Identity
   * @return shifting operator
   */
  Eigen::Matrix<Scalar, 6, 6> leftLogTransformer() const { return Eigen::Matrix<Scalar, 6, 6>::Identity(); }
};

}  // namespace dsopp::energy::motion

#endif  // DSOPP_SRC_ENERGY_MOTION_SE3_MOTION_HPP_
