#ifndef DSOPP_TRACKER_MONOCULAR_INITIALIZER_PURE_ROTATION_ESTIMATOR_HPP_
#define DSOPP_TRACKER_MONOCULAR_INITIALIZER_PURE_ROTATION_ESTIMATOR_HPP_

#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include "common/settings.hpp"

namespace dsopp {
namespace energy {
namespace n_point_solvers {

/**
 * Class to estimate pure rotation between 2 sets of bearings obtained by unprojecting 2d points
 *
 * minimizing sum (y - Rx)^T (y - Rx) <=> max sum y^TRx
 * max sum y^TRx = max trace(sum Rxy^T) = max trace(RH) = trace(RUEV^T)
 */
template <size_t kRotationNumberOfSamples, typename Model, typename Scalar = Precision>
class PureRorationSolver {
 public:
  /** number of points in the solver */
  static const int SampleSize = kRotationNumberOfSamples;
  /** dimension of input points A */
  static const int DimensionA = 2;
  /** dimension of input points B */
  static const int DimensionB = 2;
  /** Storage to return results from the solver: so3  */
  using Solution = Sophus::SO3<Scalar>;
  /**
   * Creates solver with the given camera model
   * @param model camera model
   */
  PureRorationSolver(const Model &model);
  /**
   * Estimate pure rotation
   * @param reference_points, target_points 2d points from each image
   * @return true if solution exists and false otherwise
   */
  bool solve(const Eigen::Matrix<Scalar, SampleSize, DimensionA> &reference_points,
             const Eigen::Matrix<Scalar, SampleSize, DimensionB> &target_points);

  /**
   * Return the best solution corresponding the given data with the number of inliers
   * @param reference_points, target_points data to find best solution
   * @param inlier_threshold threshold in pixels
   * @return vector of inliers and best solution
   */
  std::pair<std::vector<int>, std::optional<Solution>> findBest(
      const Eigen::Matrix<Scalar, Eigen::Dynamic, DimensionA> &reference_points,
      const Eigen::Matrix<Scalar, Eigen::Dynamic, DimensionB> &target_points, Scalar inlier_threshold = 5e-3) const;

  /**
   * @return solution stored after last solve()
   */
  const Solution &solution() const;

 private:
  /** camera model */
  const Model &model_;
  /** solutions */
  Solution solution_;
};

}  // namespace n_point_solvers
}  // namespace energy
}  // namespace dsopp

#endif  // DSOPP_TRACKER_MONOCULAR_INITIALIZER_PURE_ROTATION_ESTIMATOR_HPP_
