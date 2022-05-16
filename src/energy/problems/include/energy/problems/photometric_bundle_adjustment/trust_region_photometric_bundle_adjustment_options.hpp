#ifndef DSOPP_ENERGY_PROBLEMS_PHOTOMETRIC_BUNDLE_ADJUSTMENT_TRUST_REGION_PHOTOMETRIC_BUNDLE_ADJUSTMENT_OPTIONS_HPP_
#define DSOPP_ENERGY_PROBLEMS_PHOTOMETRIC_BUNDLE_ADJUSTMENT_TRUST_REGION_PHOTOMETRIC_BUNDLE_ADJUSTMENT_OPTIONS_HPP_

#include <Eigen/Dense>

namespace dsopp::energy::problem {

/**
 * \brief class stores trust region optimization options
 *
 * @tparam Scalar scalar type
 */
template <typename Scalar>
struct TrustRegionPhotometricBundleAdjustmentOptions {
 public:
  /**
   * @param _sigma_huber_loss constant for using in the huber loss function
   * @param _max_iterations maximum iterations for the solver
   * @param _initial_trust_region_radius refer to ceres::options.initial_trust_region_radius for details
   * @param _function_tolerance refer to ceres::options.functions_tolerance for details
   * @param _affine_brightness_regularizer affine brightness regularizer
   * @param _fixed_state_regularizer fixed state regularizer
   * @param _parameter_tolerance refer to ceres::options.parameter_tolerance for details
   */
  TrustRegionPhotometricBundleAdjustmentOptions(size_t _max_iterations, Scalar _initial_trust_region_radius,
                                                Scalar _function_tolerance, Scalar _parameter_tolerance,
                                                const Eigen::Vector2<Scalar> &_affine_brightness_regularizer,
                                                const Scalar _fixed_state_regularizer,
                                                const Scalar _sigma_huber_loss = 5)
      : max_iterations(_max_iterations),
        initial_trust_region_radius(_initial_trust_region_radius),
        function_tolerance(_function_tolerance),
        parameter_tolerance(_parameter_tolerance),
        affine_brightness_regularizer(_affine_brightness_regularizer),
        fixed_state_regularizer(_fixed_state_regularizer),
        sigma_huber_loss(_sigma_huber_loss) {}

  /** maximum iterations for the solver */
  const size_t max_iterations;
  /** refer to ceres::options.initial_trust_region_radius for details*/
  const Scalar initial_trust_region_radius;
  /** refer to ceres::options.functions_tolerance for details*/
  const Scalar function_tolerance;
  /** refer to ceres::options.parameter_tolerance for details*/
  const Scalar parameter_tolerance;
  /** affine brightness regularizer (squared inverse of prior distribution's standart deviation)*/
  const Eigen::Vector2<Scalar> affine_brightness_regularizer;
  /** constant state regularizer (used instead of fixing the pose)*/
  const Scalar fixed_state_regularizer;
  /** sigma_huber_loss constant for using in the huber loss function */
  const Scalar sigma_huber_loss;
};

}  // namespace dsopp::energy::problem

#endif  // DSOPP_ENERGY_PROBLEMS_PHOTOMETRIC_BUNDLE_ADJUSTMENT_TRUST_REGION_PHOTOMETRIC_BUNDLE_ADJUSTMENT_OPTIONS_HPP_
