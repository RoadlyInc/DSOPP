#ifndef DSOPP_LEVENBERG_MARQUARDT_ALGORITHM_HPP
#define DSOPP_LEVENBERG_MARQUARDT_ALGORITHM_HPP

#include <concepts>

#include "common/settings.hpp"

namespace dsopp::energy::levenberg_marquardt_algorithm {

/**
 * \brief CameraCalibration interface.
 *
 * CameraCalibration is an interface that stores camera model parameters. The main responsibility
 * of this interface is to produce CameraModel. Every camera calibration model should implement this interface.
 */
template <typename Problem>
concept LevenbergMarquardtProblem = requires(Problem &problem, const Precision levenberg_marquardt_regularizer) {
  /** calculation of system energy, return energy and number of valid residuals */
  { problem.calculateEnergy() }
  ->std::same_as<std::pair<Precision, int>>;
  /** linearization of the problem */
  { problem.linearize() }
  ->std::same_as<void>;
  /** saving of the current state of the problem and applying of the step. */
  { problem.calculateStep(levenberg_marquardt_regularizer) }
  ->std::same_as<void>;
  /** accept step and return state squared norm and step squared norm */
  { problem.acceptStep() }
  ->std::same_as<std::pair<Precision, Precision>>;
  /** reject step */
  { problem.rejectStep() }
  ->std::same_as<void>;
};

/**
 * The options structure contains options that control how the Levenberg-Marquardt solver operates.
 */
struct Options {
  /** Max number of iterations */
  size_t max_num_iterations = 50;
  /** Initial value of the Levenberg-Marquardt regularizer. Refer to ceres::options.initial_trust_region_radius for
   * details */
  Precision initial_levenberg_marquardt_regularizer = 1e-5_p;
  /** Refer to ceres::options.functions_tolerance for details */
  Precision function_tolerance = 1e-8_p;
  /** Refer to ceres::options.parameter_tolerance for details */
  Precision parameter_tolerance = 1e-8_p;
  /** if true accept every step until min_num_iterations reached, exit on the first unsuccesfull step after */
  bool force_accept = false;
  /** Min number of iterations only used when force_accept == true */
  size_t min_num_iterations = 0;
  /** In the case of decreasing energy, Levenberg-Marquardt regularizer will decrease. */
  Precision levenberg_marquardt_regularizer_decrease_on_accept = 2;
  /** In the case of increasing energy, Levenberg-Marquardt regularizer will increase. */
  Precision levenberg_marquardt_regularizer_increase_on_reject = 10;
};

/**
 * Output of the Levenberg-Marquardt solver after optimization.
 */
struct Result {
  /** Result energy after optimization */
  Precision energy = std::numeric_limits<Precision>::max();
  /** Number of valid residuals after optimization */
  int number_of_valid_residuals = 0;
  /** True if the problem was converged and false otherwise */
  bool converged = false;
};

/**
 * Function for solving the optimization problem by the Levenberg-Marquardt method (LM).
 *
 * @param problem the problem object
 * @param options the options object
 *
 */
template <LevenbergMarquardtProblem Problem>
Result solve(Problem &problem, const Options &options) {
  Result result;
  Precision levenberg_marquardt_regularizer = options.initial_levenberg_marquardt_regularizer;

  std::tie(result.energy, result.number_of_valid_residuals) = problem.calculateEnergy();
  bool linear_system_valid = false;

  for (size_t iteration = 0;
       iteration < options.max_num_iterations && !result.converged && result.number_of_valid_residuals > 0;
       ++iteration) {
    if (!linear_system_valid) {
      problem.linearize();
    }
    problem.calculateStep(levenberg_marquardt_regularizer);

    auto [next_energy, number_of_valid_residuals] = problem.calculateEnergy();

    if (problem.stop() || number_of_valid_residuals == 0) {
      problem.rejectStep();
      break;
    }

    bool function_tolerance_reached =
        std::abs(result.energy - next_energy) / result.energy < options.function_tolerance;
    result.converged |= function_tolerance_reached;

    if (next_energy < result.energy || (options.force_accept && iteration < options.min_num_iterations)) {
      auto [state_squared_norm, step_squared_norm] = problem.acceptStep();
      bool parameter_tolerance_reached =
          step_squared_norm < options.parameter_tolerance * (state_squared_norm + options.parameter_tolerance);
      result.converged |= parameter_tolerance_reached;

      result.energy = next_energy;
      result.number_of_valid_residuals = number_of_valid_residuals;

      levenberg_marquardt_regularizer /= options.levenberg_marquardt_regularizer_decrease_on_accept;
      linear_system_valid = false;
    } else {
      problem.rejectStep();

      if (options.force_accept) {
        problem.calculateEnergy();
        return result;
      }
      levenberg_marquardt_regularizer *= options.levenberg_marquardt_regularizer_increase_on_reject;
      linear_system_valid = true;
    }
  }
  problem.calculateEnergy();
  return result;
}
}  // namespace dsopp::energy::levenberg_marquardt_algorithm

#endif  // DSOPP_LEVENBERG_MARQUARDT_ALGORITHM_HPP
