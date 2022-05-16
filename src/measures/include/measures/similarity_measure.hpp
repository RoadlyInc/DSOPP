#ifndef DSOPP_SIMILARITY_MEASURE_HPP
#define DSOPP_SIMILARITY_MEASURE_HPP

#include "common/pattern/pattern.hpp"

#include <Eigen/Dense>
#include <concepts>

namespace dsopp {
namespace measure {
/** \brief SimilarityMeasure interface
 *
 * SimilarityMeasure is an concept for similarity measure implementation.
 *
 */
template <typename Measure, int N = Pattern::kSize>
concept SimilarityMeasure = requires(const Eigen::Vector<Precision, N>& patch) {
  { Measure::calculate(patch, patch) }
  ->std::same_as<Precision>;
};
/** \brief DifferentiableSimilarityMeasure interface
 *
 * DifferentiableSimilarityMeasure is a similarity measure can be used in optimization solvers.
 */
template <typename Measure, int N = Pattern::kSize>
concept DifferentiableSimilarityMeasure = SimilarityMeasure<Measure, N>&& requires(Eigen::Vector<Precision, N>& patch) {
  { Measure::residuals(patch, patch, patch) }
  ->std::same_as<void>;
};
}  // namespace measure
}  // namespace dsopp
#endif  // DSOPP_SIMILARITY_MEASURE_HPP
