#include "features/camera/pixel_data_frame.hpp"

#include <glog/logging.h>
#include <numeric>

#include "features/camera/downscale_image.hpp"
#include "features/camera/photometrically_corrected_image.hpp"
#include "features/camera/pixel_map.hpp"

namespace dsopp::features {

PixelDataFrame::PixelDataFrame(const cv::Mat &image, const PhotometricCalibration &photometric_calibration,
                               const Vignetting &vignetting, size_t levels) {
  levels = std::min(levels, kMaxPyramidDepth);

  // TODO Do vignetting before undistorting to avoid division by zero
  std::vector<Precision> zero_level = photometricallyCorrectedImage(image, photometric_calibration, vignetting);

  auto width = static_cast<int>(image.cols);
  auto height = static_cast<int>(image.rows);

  pyramid_.emplace_back(std::move(zero_level), width, height);
  for (size_t level = 1; level < levels; ++level) {
    std::vector<Precision> level_image = downscaleImage(pyramid_.back().data(), height, width);
    width /= 2;
    height /= 2;
    pyramid_.emplace_back(std::move(level_image), width, height);
  }
}

const PixelMap<1> &PixelDataFrame::getLevel(size_t level) const { return pyramid_.at(level); }

size_t PixelDataFrame::size() const { return pyramid_.size(); }

Pyramid movePyramidAndDelete(std::unique_ptr<PixelDataFrame> &pixel_data_frame) {
  auto tmp = std::move(pixel_data_frame->pyramid_);
  pixel_data_frame = nullptr;
  return tmp;
}

const std::vector<PixelMap<1>> &PixelDataFrame::pyramids() const { return pyramid_; }

PixelDataFrame::~PixelDataFrame() = default;
}  // namespace dsopp::features
