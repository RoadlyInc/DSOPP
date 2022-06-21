#include "features/camera/pixel_data_frame_extractor.hpp"

#include "features/camera/pixel_data_frame.hpp"

namespace dsopp {
namespace features {
PixelDataFrameExtractor::PixelDataFrameExtractor(const std::array<Precision, 256>& photometric_calibration,
                                                 const cv::Mat& undistorted_vignetting, const size_t& pyramid_levels)
    : photometric_calibration_(photometric_calibration),
      undistorted_vignetting_(undistorted_vignetting),
      pyramid_levels_(pyramid_levels) {}
std::unique_ptr<PixelDataFrame> PixelDataFrameExtractor::extract(const cv::Mat& frame) const {
  return std::make_unique<features::PixelDataFrame>(frame, photometric_calibration_, undistorted_vignetting_,
                                                    pyramid_levels_);
}
}  // namespace features
}  // namespace dsopp
