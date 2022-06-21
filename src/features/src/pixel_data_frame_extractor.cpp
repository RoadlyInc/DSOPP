#include "features/camera/pixel_data_frame_extractor.hpp"

#include "features/camera/pixel_data_frame.hpp"

namespace dsopp {
namespace features {
PixelDataFrameExtractor::PixelDataFrameExtractor(const size_t& pyramid_levels) : pyramid_levels_(pyramid_levels) {}
std::unique_ptr<PixelDataFrame> PixelDataFrameExtractor::extract(const cv::Mat& frame) const {
  return std::make_unique<features::PixelDataFrame>(frame, pyramid_levels_);
}
}  // namespace features
}  // namespace dsopp
