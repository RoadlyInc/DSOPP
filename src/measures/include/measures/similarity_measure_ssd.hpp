#ifndef DSOPP_SIMILARITY_MEASURE_SSD_HPP
#define DSOPP_SIMILARITY_MEASURE_SSD_HPP

#include "measures/similarity_measure.hpp"

namespace dsopp {
namespace measure {
/** \brief Sum Squared Differences SimilarityMeasure class
 *
 * SimilarityMeasureSSD is a similarity measure for box matching algorithm.
 * This measure is calculating by the square norm of the residual vector between patches.
 */
class SimilarityMeasureSSD {
 public:
  /**
   * calculates cost function for 2 patches (measure of difference)
   * @param patch1, patch2 patches do be compared
   * @return cost
   */
  template <typename EigenDerivedA, typename EigenDerivedB>
  static Precision calculate(const Eigen::MatrixBase<EigenDerivedA> &patch1,
                             const Eigen::MatrixBase<EigenDerivedB> &patch2) {
    return (patch1 - patch2).squaredNorm();
  }
  /**
   * residuals for Gauss-Newton optimization, targeted cost function is sum of squared residuals
   * @param patch1, patch2 patches do be compared
   * @param[out] result residuals
   */
  template <typename EigenDerivedA, typename EigenDerivedB, typename EigenDerivedC>
  static void residuals(const Eigen::MatrixBase<EigenDerivedA> &patch1, const Eigen::MatrixBase<EigenDerivedB> &patch2,
                        Eigen::MatrixBase<EigenDerivedC> &result) {
    result.noalias() = patch1 - patch2;
  }
  static_assert(DifferentiableSimilarityMeasure<SimilarityMeasureSSD, 8>);
  static_assert(SimilarityMeasure<SimilarityMeasureSSD, 8>);
};
}  // namespace measure
}  // namespace dsopp
#endif  // DSOPP_SIMILARITY_MEASURE_SSD_HPP
