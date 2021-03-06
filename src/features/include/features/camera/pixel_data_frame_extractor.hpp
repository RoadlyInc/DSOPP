#ifndef DSOPP_PIXEL_DATA_FRAME_EXTRACTOR_HPP
#define DSOPP_PIXEL_DATA_FRAME_EXTRACTOR_HPP
#include <memory>
#include <opencv2/opencv.hpp>

#include "common/settings.hpp"

namespace dsopp {
namespace sensors {
namespace providers {
class CameraDataFrame;
}
}  // namespace sensors
namespace features {
class PixelDataFrame;
/**
 * extractor to extract pyramids from image
 */
class PixelDataFrameExtractor {
 public:
  /**
   * creates extractor
   * @param photometric_calibration gamma correction to apply to image
   * @param undistorted_vignetting vignetting to apply to image
   * @param pyramid_levels number of pyramids to produce
   */
  PixelDataFrameExtractor(const std::array<Precision, 256>& photometric_calibration,
                          const cv::Mat& undistorted_vignetting, const size_t& pyramid_levels);
  /**
   * extracts pyramids from image
   * @param frame iimage to extract image
   * @return pyramids
   */
  std::unique_ptr<PixelDataFrame> extract(const cv::Mat& frame) const;

 private:
  /** photometric_calibration gamma correction to apply to image */
  const std::array<Precision, 256>& photometric_calibration_;
  /** vignetting to apply to image */
  const cv::Mat& undistorted_vignetting_;
  /** number of pyramids to produce */
  const size_t pyramid_levels_;
};
}  // namespace features
}  // namespace dsopp

#endif  // DSOPP_PIXEL_DATA_FRAME_EXTRACTOR_HPP
