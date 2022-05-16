#include "sensors/camera_calibration/undistorter/undistorter.hpp"

#include <glog/logging.h>

namespace dsopp::sensors::calibration {

cv::Mat Undistorter::undistort(const cv::Mat &img, int interpolationType) const {
  if (remapX_.empty() || remapY_.empty()) {
    return img.clone();
  }
  if (static_cast<int>(input_width_) != img.cols or static_cast<int>(input_height_) != img.rows) {
    LOG(ERROR) << "Incompatible image size to remap size " << img.cols << " " << img.rows;
  }

  cv::Mat dst;
  cv::remap(img, dst, remapX_, remapY_, interpolationType, 4);
  return dst;
}

Undistorter Undistorter::Identity(const Eigen::Vector2<Precision> &image_size) {
  return Undistorter(cv::Mat(), cv::Mat(), static_cast<size_t>(image_size.x()), static_cast<size_t>(image_size.y()));
}

size_t Undistorter::input_width() const { return input_width_; }

size_t Undistorter::input_height() const { return input_height_; }

}  // namespace dsopp::sensors::calibration
