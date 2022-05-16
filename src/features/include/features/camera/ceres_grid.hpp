#ifndef DSOPP_SRC_FEATURES_CAMERA_CERES_GRID_H_
#define DSOPP_SRC_FEATURES_CAMERA_CERES_GRID_H_

#include <ceres/cubic_interpolation.h>

#include "common/constexpr_tools/constexpr_tools.hpp"
#include "common/patch/patch.hpp"
#include "pixel_map.hpp"

namespace dsopp {
namespace features {
/**
 * \brief CeresGrid is the continuous ceres 2D grid which contains information from the image.
 *
 * CeresGrid contains information from the image and cubically interpolates it in the 2D grid.
 */
template <int C>
class CeresGrid {
  /** shortcut for the ceres 3D grid */
  using Grid = ceres::Grid2D<double, C>;

 public:
  /**
   * creates composition with the ceres interpolator
   * @param grid grid for the interpolator
   */
  explicit CeresGrid(const features::PixelMap<C>& grid)
      : data_(grid.data().begin(), grid.data().end()),
        grid_(&data_[0], 0, static_cast<int>(grid.height()), 0, static_cast<int>(grid.width())),
        interpolator_(ceres::BiCubicInterpolator<Grid>(grid_)) {}
  /**
   * method to get the interpolated data from the map
   *
   * @param r, c coordinates of the data (notation from ceres)
   * @param[out] f interpolated data with derivatives
   */
  template <typename T>
  void Evaluate(const T& r, const T& c, T* f) const {
    interpolator_.Evaluate(r, c, f);
  }
  /**
   * method to get the interpolated pixel from the map
   *
   * @param r, c coordinates of the data (notation from ceres)
   * @param[out] f interpolated data
   */
  void Evaluate(const double& r, const double& c, PixelInfo<C>* f) const {
    Eigen::Matrix<double, C, 1> frc, dfdr, dfdc;

    interpolator_.Evaluate(r, c, frc.data(), dfdr.data(), dfdc.data());
    f->intensity() = frc;
    f->jacobian().col(0) = dfdc;
    f->jacobian().col(1) = dfdr;
  }
  /**
   * method to interpolate pixel coordiantes at N locations
   *
   * @tparam T scalar type
   * @tparam N number of pixel coordinates
   *
   * @param pattern columns of locationts to evaluate in
   * @param[out] patch evaluated pixel values
   */
  template <typename T, int N>
  void Evaluate(const Eigen::Matrix<T, 2, N>& pattern, Eigen::Matrix<T, N, C, PatchStorageOrder<C>>& patch) const {
    common::constexpr_tools::constexpr_for<0, N, 1>(
        [&]<int I>() { Evaluate(pattern(1, I), pattern(0, I), patch.row(I).data()); });
  }
  /**
   * method to interpolate pixel coordiantes at N locations for float type
   *
   * used mostly for tests
   *
   * @tparam N number of pixel coordinates
   *
   * @param pattern columns of locationts to evaluate in
   * @param[out] patch evaluated pixel values
   */
  template <int N>
  void Evaluate(const Eigen::Matrix<float, 2, N>& pattern,
                Eigen::Matrix<float, N, C, PatchStorageOrder<C>>& patch) const {
    Eigen::Matrix<double, 2, N> pattern_double = pattern.template cast<double>();
    Eigen::Matrix<double, N, C, PatchStorageOrder<C>> patch_double;
    Evaluate(pattern_double, patch_double);
    patch = patch_double.template cast<float>();
  }
  /**
   * method to interpolate pixel coordiantes and jacobians at N locations
   *
   * @tparam N number of pixel coordinates
   *
   * @param pattern columns of locationts to evaluate in
   * @param[out] patch evaluated pixel values
   * @param[out] d_intensity_u_diag,d_intensity_v_diag diagonals of jacobian matrices i.e d_intensity_u_diag(i)  =
   * d(u_i)/d(ididepth_i)
   */
  template <int N>
  void Evaluate(const Eigen::Matrix<double, 2, N>& pattern, Eigen::Matrix<double, N, C, PatchStorageOrder<C>>& patch,
                Eigen::Vector<double, N * C>& d_intensity_u_diag,
                Eigen::Vector<double, N * C>& d_intensity_v_diag) const {
    common::constexpr_tools::constexpr_for<0, N, 1>([&]<int I>() {
      interpolator_.Evaluate(pattern(1, I), pattern(0, I), patch.row(I).data(), d_intensity_v_diag.data() + I * C,
                             d_intensity_u_diag.data() + I * C);
    });
  }
  /**
   * method to interpolate pixel coordiantes and jacobians at N locations for float type
   *
   * used mostly for testing
   *
   * @tparam N number of pixel coordinates
   *
   * @param pattern columns of locationts to evaluate in
   * @param[out] patch evaluated pixel values
   * @param[out] d_intensity_u_diag,d_intensity_v_diag diagonals of jacobian matrices i.e d_intensity_u_diag(i)  =
   * d(u_i)/d(ididepth_i)
   */
  template <int N>
  void Evaluate(const Eigen::Matrix<float, 2, N>& pattern, Eigen::Matrix<float, N, C, PatchStorageOrder<C>>& patch,
                Eigen::Vector<float, N * C>& d_intensity_u_diag,
                Eigen::Vector<float, N * C>& d_intensity_v_diag) const {
    Eigen::Matrix<double, 2, N> pattern_double = pattern.template cast<double>();
    Eigen::Matrix<double, N, C, PatchStorageOrder<C>> patch_double;
    Eigen::Vector<double, N * C> d_intensity_u_diag_double;
    Eigen::Vector<double, N * C> d_intensity_v_diag_double;
    Evaluate(pattern_double, patch_double, d_intensity_u_diag_double, d_intensity_v_diag_double);
    patch = patch_double.template cast<float>();
    d_intensity_u_diag = d_intensity_u_diag_double.template cast<float>();
    d_intensity_v_diag = d_intensity_v_diag_double.template cast<float>();
  }

 private:
  // we need to copy data here, as we keep data float, but ceres only accept double
  /** storage for the grid data */
  const std::vector<double> data_;
  /** grid for the ceres interpolator */
  const Grid grid_;
  /** ceres interpolator */
  const ceres::BiCubicInterpolator<Grid> interpolator_;
};
}  // namespace features
}  // namespace dsopp

#endif  // DSOPP_SRC_FEATURES_CAMERA_CERES_GRID_H_
