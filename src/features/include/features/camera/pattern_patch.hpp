#ifndef DSOPP_PATTERN_PATCH_HPP
#define DSOPP_PATTERN_PATCH_HPP

#include "common/patch/patch.hpp"
#include "common/pattern/pattern.hpp"
#include "sensors/camera_calibration/mask/camera_mask.hpp"

#include <cstddef>
#include <functional>
#include <type_traits>

#include <Eigen/Dense>
#include <sophus/se3.hpp>

namespace dsopp::features {
/** \brief Pattern patch number 4 class
 *
 * PatternPatch is used in conjunction with SimilarityMeasure for e.g. coarse depth estimation.
 * For details, see paper 'Direct Sparse Odometry' by Engel et al.
 * https://arxiv.org/pdf/1607.02565.pdf
 */
class PatternPatch {
 public:
  /** alias for number of points on the pattern */
  static constexpr int N = Pattern::kSize;
  /**
   * method to interpolate the pixel patch around the given point
   * @param point given point
   * @param grid 2D grid to get interpolated values
   * @param[out] pixel_patch interpolated pixel patch
   * */
  template <typename Grid, typename Scalar, typename ResultScalar = Scalar, int PatternSize = N>
  static void getIntensities(const Eigen::Vector2<Scalar> &point, const Grid &grid,
                             Eigen::Vector<ResultScalar, PatternSize> &pixel_patch) {
    static_assert(PatternSize != N || PatternSize != 1, "Can't evaluate pixel patch size ");

    if constexpr (PatternSize == N) {
      auto pattern_coordinates =
          Eigen::Map<Pattern::EigenType>(Pattern::pattern_data).template cast<Scalar>().colwise() + point;
      for (int idx = 0; idx < N; ++idx) {
        Eigen::Vector2<Scalar> coord = pattern_coordinates.col(idx);
        grid.Evaluate(coord(1), coord(0), &pixel_patch(idx));
      }
    } else if constexpr (PatternSize == 1) {
      grid.Evaluate(point(1), point(0), &pixel_patch[0]);
    }
  }

  /**
   * method to interpolate the patch around the given point
   * @param point given point
   * @param grid 2D grid to get interpolated values
   * @param[out] patch interpolated patch
   * */
  template <typename Grid, typename Scalar, typename ResultScalar = Scalar, int PatternSize = N, int C>
  static void getIntensities(const Eigen::Vector2<Scalar> &point, const Grid &grid,
                             Eigen::Matrix<ResultScalar, PatternSize, C, PatchStorageOrder<C>> &patch) {
    static_assert(PatternSize != N || PatternSize != 1, "Can't evaluate patch size ");

    if constexpr (PatternSize == N) {
      auto pattern_coordinates =
          Eigen::Map<Pattern::EigenType>(Pattern::pattern_data).template cast<Scalar>().colwise() + point;
      grid.Evaluate(pattern_coordinates, patch);
    } else if constexpr (PatternSize == 1) {
      grid.Evaluate(point, patch);
    }
  }

  /**
   * method to shift pattern by point
   *
   * @param point center of the pattern
   * @param[out] shifted shifted pattern coordinates
   */
  template <typename EigenDerivedA, typename EigenDerivedB>
  static void shiftPattern(const Eigen::MatrixBase<EigenDerivedA> &point,
                           Eigen::MatrixBase<EigenDerivedB> const &shifted) {
    const_cast<Eigen::MatrixBase<EigenDerivedB> &>(shifted).noalias() =
        Eigen::Map<Pattern::EigenType>(Pattern::pattern_data)
            .template cast<typename EigenDerivedB::Scalar>()
            .colwise() +
        point;
  }
};

}  // namespace dsopp::features
#endif  // DSOPP_PATTERN_PATCH_HPP
