#ifndef DSOPP_PATTERN_COMMON_HPP_
#define DSOPP_PATTERN_COMMON_HPP_

#include <Eigen/Dense>

#include "common/settings.hpp"

namespace dsopp {
namespace patterns {
/**
 * \brief Dso 8 point neighbourhood pattern image
 * For details, see paper 'Direct Sparse Odometry' by Engel et al.
 * https://arxiv.org/pdf/1607.02565.pdf
 */
struct EightPointPattern {
  /** number of neighbours points in pattern image */
  static constexpr int kSize = 8;
  /** position of the center in the patch */
  static constexpr int kCenter = 4;
  /** raw pattern data in x_i, y_i order */
  static constexpr Precision pattern_data[kSize * 2] = {
      // clang-format off
      0,  2, 
     -1,  1, 
      1,  1, 
     -2,  0, 
      0,  0, 
      2,  0, 
     -1, -1, 
      0, -2
      // clang-format on
  };
  /** alias for pattern Eigen type */
  using EigenType = const Eigen::Matrix<Precision, 2, kSize>;
};
}  // namespace patterns
/** alias for main pattern in all slam library */
using Pattern = patterns::EightPointPattern;
}  // namespace dsopp

#endif  // DSOPP_PATTERN_COMMON_HPP_
