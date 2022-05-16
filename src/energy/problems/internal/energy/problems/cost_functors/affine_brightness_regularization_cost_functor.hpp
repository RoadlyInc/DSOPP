#ifndef DSOPP_SRC_ENERGY_PROBLEMS_COST_FUNCTORS_AFFINE_BRIGHTNESS_REGULARIZATION_COST_FUNCTOR_HPP_
#define DSOPP_SRC_ENERGY_PROBLEMS_COST_FUNCTORS_AFFINE_BRIGHTNESS_REGULARIZATION_COST_FUNCTOR_HPP_

#include <Eigen/Dense>

namespace dsopp {
namespace energy {
namespace problem {
namespace cost_functors {
/** \brief Functor to calculate the cost of affine brightness regularization
 *  @tparam ParameterOffset offset of affine parameters in `state_eps_ptr` array
 * */
template <size_t ParameterOffset>
class AffineBrightnessRegularizationCostFunctor {
 public:
  /** number of residuals */
  static constexpr int residuals_num = 2;
  /**
   * Create functor to calculate the cost of affine brightness regularization
   * @param regularizer regularizer affine brightess at first iteration
   * @param affine_brightness0
   * */
  AffineBrightnessRegularizationCostFunctor(const Eigen::Vector2d &regularizer,
                                            const Eigen::Vector2d &affine_brightness0)
      : regularizer_(regularizer), affine_brightness0_(affine_brightness0) {}
  /**
   * Operator overloading for the ceres optimization
   * @param state_eps_ptr state eps
   * @param residuals_ptr cost to fill out
   * @return true
   * */
  template <class T>
  bool operator()(T const *const state_eps_ptr, T *residuals_ptr) const {
    Eigen::Map<Eigen::Matrix<T, 2, 1> const> const affine_brightness_eps(state_eps_ptr + ParameterOffset);
    Eigen::Map<Eigen::Matrix<T, residuals_num, 1>> residuals(residuals_ptr);

    residuals = regularizer_.cast<T>().cwiseProduct(affine_brightness0_.cast<T>() + affine_brightness_eps);
    return true;
  }

 private:
  /** regularizer */
  const Eigen::Vector2d regularizer_;
  /** affine brightess at first iteration */
  const Eigen::Vector2d affine_brightness0_;
};
}  // namespace cost_functors
}  // namespace problem
}  // namespace energy
}  // namespace dsopp

#endif  // DSOPP_SRC_ENERGY_PROBLEMS_COST_FUNCTORS_AFFINE_BRIGHTNESS_REGULARIZATION_COST_FUNCTOR_HPP_
