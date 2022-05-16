#ifndef DSOPP_PIXELMAP_H
#define DSOPP_PIXELMAP_H

#include <vector>

#include <ceres/jet.h>
#include <Eigen/Dense>

#include "common/constexpr_tools/constexpr_tools.hpp"
#include "common/settings.hpp"

namespace dsopp {
namespace features {

template <int C>
class PixelMap;

namespace {
template <bool WITH_JACOBIANS, int C>
__attribute__((always_inline)) inline auto interpolateLinear(const Precision x, const Precision y,
                                                             const typename PixelMap<C>::PixelInfoStorage &map) {
  const int ix = static_cast<int>(x);
  const int iy = static_cast<int>(y);
  const Precision dx = x - static_cast<Precision>(ix);
  const Precision dy = y - static_cast<Precision>(iy);
  const Precision dxdy = dx * dy;

  if constexpr (WITH_JACOBIANS) {
    // clang-format off
    return dxdy                 * map(iy + 1, ix + 1).data_ +
           (dy - dxdy)          * map(iy + 1, ix    ).data_ +
           (dx - dxdy)          * map(iy    , ix + 1).data_ +
           (1 - dx - dy + dxdy) * map(iy    , ix    ).data_;
    // clang-format on
  } else {
    return dxdy * map(iy + 1, ix + 1).data_.col(0) + (dy - dxdy) * map(iy + 1, ix).data_.col(0) +
           (dx - dxdy) * map(iy, ix + 1).data_.col(0) + (1 - dx - dy + dxdy) * map(iy, ix).data_.col(0);
  }
}

}  // namespace

template <int C>
struct PixelInfo;

namespace internal {
template <typename T>
struct traits;

/**
 * \brief traits<PixelInfo> needed to have different return types for PixelInfo implementation
 */
template <int C>
struct traits<PixelInfo<C>> {
  /** Return type for C channel image*/
  using PixelReturnType = Eigen::Ref<Eigen::Matrix<Precision, C, 1>>;
  /** Return const type for C channel image*/
  using PixelReturnTypeConst = Eigen::Ref<const Eigen::Matrix<Precision, C, 1>>;
};

/**
 * \brief traits<PixelInfo> specialization for image with 1 channel
 */
template <>
struct traits<PixelInfo<1>> {
  /** Return type for 1 channel image*/
  using PixelReturnType = Precision &;
  /** Return const type for 1 channel image*/
  using PixelReturnTypeConst = const Precision;
};

}  // namespace internal

/**
 * \brief PixelInfo stores information of individual pixel. It stores intensity,
 * derivatives
 */
template <int C>
struct PixelInfo {
  /** Return type of information for all channels*/
  using PixelReturnType = typename internal::traits<PixelInfo<C>>::PixelReturnType;
  /** Return const type of information for all channels*/
  using PixelReturnTypeConst = typename internal::traits<PixelInfo<C>>::PixelReturnTypeConst;

  /**
   * struct constructor by default
   * */
  PixelInfo() {}

  /**
   *  method to get only intensities
   *  @return Eigen Vector with intensity for each channel
   */
  PixelReturnType intensity() {
    if constexpr (C == 1)
      return data_(0, 0);
    else
      return data_.col(0);
  }
  /**
   *  method to get only intensities
   *  @return Eigen Vector with intensity for each channel
   */
  PixelReturnTypeConst intensity() const {
    if constexpr (C == 1)
      return data_(0, 0);
    else
      return data_.col(0);
  }

  /**
   * method to get Jacobian (C x 2 matrix with numerical partial derivatives)
   * @return Jacobian of image as function X x Y -> R^C
   */
  Eigen::Ref<Eigen::Matrix<Precision, C, 2>> jacobian() { return data_.template block<C, 2>(0, 1); }

  /**
   * method to get Jacobian (C x 2 matrix with numerical partial derivatives)
   * @return Jacobian of image as function X x Y -> R^C
   */
  Eigen::Matrix<Precision, C, 2> jacobian() const { return data_.template block<C, 2>(0, 1); }

  /** storage, contains intensity derivatives and gradients magnitudes
   *  col 0 -- intensity
   *  col 1 -- dx derivative
   *  col 2 -- dy derivative
   * */
  Eigen::Matrix<Precision, C, 3> data_;
};

/**
 * \brief PixelMap is C continuous grids for storing C-channel image
 *
 * Contains information about intensities, derivatives
 * Interpolates with bi-linear scheme given x,y coordinates
 *
 */
template <int C>
class PixelMap {
 public:
  /** Alias for storage type for Pixel Info */
  using PixelInfoStorage = Eigen::Array<PixelInfo<C>, -1, -1, Eigen::RowMajor>;

  /**
   * creates PixelMap from plain data and size of map
   * @param data vector with intensities of size C * width * height, data should
   * be in form of [C][height][width] RowMajor and stores each channel
   * individually and continuous
   * @param width width of the map
   * @param height height of the map
   */
  PixelMap(std::vector<Precision> &&data, long width, long height);
  /** default constructor */
  PixelMap();
  /** default move constructor */
  PixelMap(PixelMap &&);
  /** default move operator
   * @return moved object
   */
  PixelMap &operator=(PixelMap &&);
  /**
   * method to get the map width
   *
   * @return map width
   */
  long width() const;
  /**
   * method to get the map height
   *
   * @return map height
   */
  long height() const;
  /**
   * method to get the map channels size
   *
   * @return map channels
   */
  int channels() const;
  /**
   * method to get the map height*width
   *
   * @return map size = height*width
   */
  size_t size() const;
  /**
   * method to get data from the map as the vector
   *
   * @return full data from the map
   * */
  const std::vector<Precision> &data() const;

  /**
   * method to get PixelInfo for individual pixel
   *
   * @param x x-coordinate of pixel
   * @param y y-coordinate of pixel
   * @return const reference to PixelInfo object at (rows = y, cols = x)
   * location
   */
  const PixelInfo<C> &operator()(int x, int y) const;
  /**
   * method to get PixelInfo for individual pixel
   *
   * @param index ColMajor index of a pixel
   * @return reference to an element
   */
  PixelInfo<C> &operator()(size_t index);
  /**
   * method to get PixelInfo for individual pixel
   *
   * @param index ColMajor index of a pixel
   * @return const reference to an element
   */
  const PixelInfo<C> &operator()(size_t index) const;

  /**
   * method to get the interpolated intensities from the map
   *
   * @param r, c coordinates of the point (notation from ceres)
   * @param[out] f interpolated intensities (should be pointer to array of size C
   *
   */
  template <typename Scalar>
  void Evaluate(Scalar r, Scalar c, Scalar *f) const {
    Eigen::Map<Eigen::Matrix<Scalar, C, 1>> res(f);
    res =
        interpolateLinear<false, C>(static_cast<Precision>(c), static_cast<Precision>(r), map_).template cast<Scalar>();
  }
  /**
   * method to get the interpolated PixelInfo from the map
   *
   * @param r, c coordinates of the data (notation from ceres)
   * @param[out] f interpolated PixelInfo (should be pointer to valid PixelInfo)
   */
  void Evaluate(Precision r, Precision c, PixelInfo<C> *f) const { f->data_ = interpolateLinear<true, C>(c, r, map_); }
  /**
   * method to get the interpolated data with derivatives from the map
   *
   * @param r row idx of image
   * @param c column idx of image
   * @param[out] f_ interpolated data with derivatives (Should be pointer to data of size C)
   */
  template <typename Scalar, int JetN>
  void Evaluate(const ceres::Jet<Scalar, JetN> &r, const ceres::Jet<Scalar, JetN> &c,
                ceres::Jet<Scalar, JetN> *f_) const {
    Eigen::Map<Eigen::Matrix<ceres::Jet<Scalar, JetN>, C, 1>> f(f_);

    Eigen::Matrix<Scalar, C, 3> res =
        interpolateLinear<true, C>(static_cast<Precision>(c.a), static_cast<Precision>(r.a), map_)
            .template cast<Scalar>();

    common::constexpr_tools::constexpr_for<0, C, 1>([&]<int I>() {
      f(I).a = res(I, 0);
      f(I).v = res(I, 1) * c.v + res(I, 2) * r.v;
    });
  }
  /**
   * method to interpolate pixel coordinates at N locations
   *
   * @tparam T scalar type
   * @tparam N number of pixel coordinates
   *
   * @param pattern columns of locations to evaluate in
   * @param[out] patch evaluated pixel values
   */
  template <typename EigenDerivedA, typename EigenDerivedB>
  void Evaluate(const Eigen::MatrixBase<EigenDerivedA> &pattern, Eigen::MatrixBase<EigenDerivedB> &patch) const {
    static_assert(EigenDerivedA::RowsAtCompileTime == 2);
    const int N = EigenDerivedA::ColsAtCompileTime;
    static_assert(EigenDerivedB::RowsAtCompileTime == N);
    static_assert(EigenDerivedB::ColsAtCompileTime == C);

    static_assert(EigenDerivedB::IsRowMajor or C == 1);

    common::constexpr_tools::constexpr_for<0, N, 1>(
        [&]<int I>() { Evaluate(pattern(1, I), pattern(0, I), patch.row(I).data()); });
  }
  /**
   * method to interpolate pixel coordinates at N locations
   *
   * @tparam T scalar type
   * @tparam N number of pixel coordinates
   *
   * @param pattern columns of locations to evaluate in
   * @param[out] patch evaluated pixel values
   * @param[out] d_intensity_u_diag,d_intensity_v_diag diagonals of jacobian matrices i.e d_intensity_u_diag(i)  =
   * d(u_i)/d(idepth_i)
   */

  template <typename EigenDerivedA, typename EigenDerivedB, typename EigenDerivedC, typename EigenDerivedD>
  void Evaluate(const Eigen::MatrixBase<EigenDerivedA> &pattern, Eigen::MatrixBase<EigenDerivedB> &patch,
                Eigen::MatrixBase<EigenDerivedC> &d_intensity_u_diag,
                Eigen::MatrixBase<EigenDerivedD> &d_intensity_v_diag) const {
    static_assert(EigenDerivedA::RowsAtCompileTime == 2);
    const int N = EigenDerivedA::ColsAtCompileTime;
    static_assert(EigenDerivedB::RowsAtCompileTime == N);
    static_assert(EigenDerivedB::ColsAtCompileTime == C);
    static_assert(EigenDerivedC::ColsAtCompileTime == 1 and EigenDerivedD::ColsAtCompileTime == 1);
    static_assert(EigenDerivedC::RowsAtCompileTime == N * C and EigenDerivedD::RowsAtCompileTime == N * C);

    common::constexpr_tools::constexpr_for<0, N, 1>([&]<int I>() {
      const auto data = interpolateLinear<true, C>(static_cast<Precision>(pattern(0, I)),
                                                   static_cast<Precision>(pattern(1, I)), map_);
      patch.row(I) = data.col(0).template cast<typename EigenDerivedB::Scalar>();
      d_intensity_u_diag.template segment<C>(I * C) = data.col(1).template cast<typename EigenDerivedC::Scalar>();
      d_intensity_v_diag.template segment<C>(I * C) = data.col(2).template cast<typename EigenDerivedD::Scalar>();
    });
  }

  /**
   * clone object
   * @return  cloned object
   */
  PixelMap<C> clone() const;

 protected:
  /** delete copy contructor */
  PixelMap(const PixelMap &);

 private:
  /** Container with all image pixels [channel][height][width]  */
  std::vector<Precision> plain_data_;

  /** Container with channel info for all pixels  */
  PixelInfoStorage map_;
};

/** Aliases for one channel Image */
using Image1C = PixelMap<1>;
using Image1CPixelInfo = PixelInfo<1>;

}  // namespace features
}  // namespace dsopp
#endif  // DSOPP_PIXELMAP_H
