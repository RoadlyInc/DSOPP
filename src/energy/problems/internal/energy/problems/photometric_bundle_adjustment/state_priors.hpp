#ifndef DSOPP_ENERGY_PROBLEMS_PHOTOMETRIC_BUNDLE_ADJUSTMENT_STATE_PRIORS_HPP_
#define DSOPP_ENERGY_PROBLEMS_PHOTOMETRIC_BUNDLE_ADJUSTMENT_STATE_PRIORS_HPP_

#include <ceres/normal_prior.h>
#include <ceres/problem.h>
#include <Eigen/Dense>

#include "common/settings.hpp"
#include "common/time/time.hpp"
#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"

#include "energy/normal_linear_system.hpp"
#include "energy/problems//cost_functors/motion_priors.hpp"

namespace dsopp::energy::problem::hessian_prior {

/**
 * \brief templated class for Motion priors
 * @tparam Motion motion type
 */
template <motion::Motion Motion>
class MotionPrior {};

/**
 * \brief SE3 specialization of MotionPrior
 *
 * this class does nothing
 */
template <typename Scalar>
class MotionPrior<motion::SE3<Scalar>> {
  /** alias for SE3 DoF */
  static int constexpr DoF = motion::SE3<Scalar>::DoF;
  /** alias for Motion */
  using Motion = motion::SE3<Scalar>;

 public:
  /**
   * evaluate energy term of prior
   *
   * @return energy term
   */
  template <class CameraModel>
  static Scalar energyTerm(const typename Motion::Product &, const CameraModel &, const time::duration &) {
    return 0;
  }

  /**
   * update Hessian and b based on prior form
   *
   * @return prirors for reference and target frames
   */
  template <class CameraModel>
  static std::pair<NormalLinearSystem<Scalar, Motion::DoF>, NormalLinearSystem<Scalar, Motion::DoF>> systemPrior(
      const typename Motion::Product &, const typename Motion::Product, const CameraModel &, const time::duration &) {
    NormalLinearSystem<Scalar, Motion::DoF> prior_reference, prior_target;
    prior_reference.setZero();
    prior_target.setZero();
    return {prior_reference, prior_target};
  }

  /**
   * add ceres prior to the ceres problem
   */
  static void addCeresPrior(ceres::Problem &, Eigen::Vector<double, DoF + 2> &) {}
  /**
   * add relative motion prior cost to ceres problem
   */
  static void addCeresRelativeMotionPrior(ceres::Problem &, const Eigen::Vector<double, motion::SE3<double>::DoF + 2> &,
                                          const Eigen::Vector<double, motion::SE3<double>::DoF + 2> &,
                                          const time::duration &, const Eigen::Vector2<Precision> &,
                                          const time::duration &) {}
};

/**
 * \brief prior for affine brightness
 */
class AffineBrightnessPrior {
 public:
  /**
   * evaluates energy term for regularization
   *
   * @param state current parameter state
   * @param affine_brightness_regularizer regularizer for affine brightness
   * @return energy term
   */
  static Precision energyTerm(const Eigen::Vector<Precision, 2> &state,
                              const Eigen::Vector<Precision, 2> &affine_brightness_regularizer) {
    return (state.array() * affine_brightness_regularizer.array()).matrix().dot(state) / 2;
  }

  /**
   * updates Hessian and b based of prior
   *
   * @param state current parameter state
   * @param affine_brightness_regularizer regularizer for affine brightness
   *
   * @return prior system
   */
  static NormalLinearSystem<Precision, 2> priorSystem(
      const Eigen::Vector<Precision, 2> &state, const Eigen::Vector<Precision, 2> &affine_brightness_regularizer) {
    NormalLinearSystem<Precision, 2> system;
    system.H = affine_brightness_regularizer.asDiagonal();
    system.b = affine_brightness_regularizer.cwiseProduct(state);
    return system;
  }
};

}  // namespace dsopp::energy::problem::hessian_prior

#endif  // DSOPP_ENERGY_PROBLEMS_PHOTOMETRIC_BUNDLE_ADJUSTMENT_STATE_PRIORS_HPP_
