#ifndef DSOPP_CAMERA_MASK_HPP
#define DSOPP_CAMERA_MASK_HPP

#include "common/settings.hpp"
#include "semantics/semantic_filter.hpp"
#include "sensors/camera_calibration/undistorter/undistorter.hpp"

#include <Eigen/Dense>
#include <opencv2/core.hpp>

namespace dsopp::sensors::calibration {

/**
 * \brief CameraMask class, stores mask to exclude some feature points
 *
 * mask -- unsigned char 2d array with zeros in excluding region (255 otherwise)
 *
 * Mask is divided to static and dymainc, static mask is constant during execution, dynamic mask can be changed (sky
 * excluding mask)
 */
class CameraMask {
 public:
  /** Alias for Mask Storage type */
  using MaskStorage = cv::Mat;
  /** Alias for opencv cv::Mat type */
  static constexpr int MaskStorageType = CV_8UC1;
  /**
   * Constructor
   *
   * @param mask static mask
   */
  CameraMask(const MaskStorage &mask);

  /**
   * Constructor, creates white image mask
   *
   * @param rows rows
   * @param cols cols
   */
  CameraMask(int rows, int cols);
  /**
   * Method to check if input point is in mask ROI
   *
   * @param x x coordinate
   * @param y y coordinate
   * @return true if valid
   */
  template <bool CHECK_BORDERS = true>
  bool valid(const int x, const int y) const {
    if constexpr (CHECK_BORDERS) {
      if (x < 0 || x >= mask_.cols) return false;
      if (y < 0 || y >= mask_.rows) return false;
    }
    return mask_.at<uchar>(y, x);
  }
  /**
   * Method to check if input point is in mask ROI
   *
   * @param x x coordinate
   * @param y y coordinate
   * @return true if valid
   */
  template <bool CHECK_BORDERS = true>
  bool valid(const Precision x, const Precision y) const {
    return valid(static_cast<int>(std::round(x)), static_cast<int>(std::round(y)));
  }
  /**
   * Method to check if pattern is in valid roi
   *
   * @param pattern columns of point coordinates
   * @return true if all points are valid
   */
  template <bool CHECK_BORDERS = true, typename EigenDerived>
  bool valid(const Eigen::MatrixBase<EigenDerived> &pattern) const {
    static_assert(EigenDerived::RowsAtCompileTime == 2);
    const int N = EigenDerived::ColsAtCompileTime;
    using T = typename EigenDerived::Scalar;
    for (int i = 0; i < N; ++i) {
      if constexpr (std::is_arithmetic_v<T>) {
        if (!valid<CHECK_BORDERS>(static_cast<Precision>(pattern(0, i)), static_cast<Precision>(pattern(1, i))))
          return false;
      } else {
        Eigen::Vector2<T> point = pattern.col(i);
        if (!valid<CHECK_BORDERS>(static_cast<Precision>(point.x().a), static_cast<Precision>(point.y().a)))
          return false;
      }
    }
    return true;
  }
  /**
   * Method returns data
   *
   * @return CameraMask data
   */
  const MaskStorage &data() const;

  /**
   * Method return opencv compatible mask
   *
   * @return opencv compatible mask
   */
  cv::Mat openCVCompatibleMask() const;

  /**
   * Method which return delated CameraMask
   * @param border_size to perforom circular erosion
   * @return CameraMask instance
   */
  CameraMask getEroded(const int border_size) const;

  /**
   * Method which return CameraMask with filtered semantic objects
   * @param semantics_data semantics data from the frame
   * @param semantic_filter semantics filter
   * @return CameraMask with removed dynamic objects
   */
  CameraMask filterSemanticObjects(const cv::Mat &semantics_data,
                                   const semantics::SemanticFilter &semantic_filter) const;

  /**
   * Method which return resized CameraMask
   * @param scale resizing scale
   * @return CameraMask resized mask
   */
  CameraMask resize(const Precision scale) const;

 private:
  /** static mask data */
  MaskStorage mask_;
};

}  // namespace dsopp::sensors::calibration

#endif  // DSOPP_CAMERA_MASK_HPP
