#ifndef DSOPP_RANSAC_RANSAC_HPP
#define DSOPP_RANSAC_RANSAC_HPP

#include <Eigen/Dense>

#include <memory>
#include <vector>

#include "ransac/random_sequence_generator.hpp"

namespace dsopp::ransac {

/**
 * \brief Random sample consensus (RANSAC) method to estimate model using solver and set points.
 * This method used to estimate transformation between frames and intrinsic camera parameters from
 * 2d-2d od 3d-2d point correspondences.
 *
 * @param solver solver to estimate model
 * @param points_a, points_b point correspondences
 * @param inlier_threshold threshold in pixels
 * @param max_iterations_number max number of iterations in ransca
 * @param inliers_tolerance then ratio between inliers and all point become greater then the
 *
 * @tparam Solver solver using ti estimate model
 * @return vector of inlier indices and best solution
 */
template <typename Scalar, typename Solver>
std::pair<std::vector<int>, std::optional<typename Solver::Solution>> ransac(
    Solver &solver, const Eigen::Matrix<Scalar, Eigen::Dynamic, Solver::DimensionA> &points_a,
    const Eigen::Matrix<Scalar, Eigen::Dynamic, Solver::DimensionB> &points_b, const Scalar inlier_threshold,
    const size_t max_iterations_number = 1e4, const Scalar inliers_tolerance = Scalar(0.95)) {
  assert(points_a.rows() == points_b.rows());
  auto number_of_points = static_cast<size_t>(points_a.rows());
  std::vector<int> best_inliers;
  typename Solver::Solution best_solution;
  for (size_t i = 0;
       i < max_iterations_number &&
       static_cast<Scalar>(best_inliers.size()) / static_cast<Scalar>(number_of_points) < inliers_tolerance;
       i++) {
    auto sample = generateRandomSequence<Solver::SampleSize>(number_of_points);
    Eigen::Matrix<Scalar, Solver::SampleSize, Solver::DimensionA> sample_a;
    Eigen::Matrix<Scalar, Solver::SampleSize, Solver::DimensionB> sample_b;
    for (int j = 0; j < Solver::SampleSize; j++) {
      sample_a.row(j) = points_a.row(static_cast<int>(sample[static_cast<size_t>(j)]));
      sample_b.row(j) = points_b.row(static_cast<int>(sample[static_cast<size_t>(j)]));
    }
    solver.solve(sample_a, sample_b);
    auto [inliers, solution] = solver.findBest(points_a, points_b, inlier_threshold);
    if (solution.has_value() && inliers.size() > best_inliers.size()) {
      best_inliers = std::move(inliers);
      best_solution = *solution;
    }
  }

  return {best_inliers, best_solution};
}

}  // namespace dsopp::ransac

#endif  // DSOPP_RANSAC_RANSAC_HPP
