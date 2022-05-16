#include "sensors/camera_transformers/camera_resizer.hpp"

namespace dsopp::sensors::camera_transformers {

CameraResizer::CameraResizer(const Precision &new_to_old_size_ratio) : new_to_old_size_ratio_(new_to_old_size_ratio) {}

void CameraResizer::transformImage(cv::Mat &image, int interpolation_type) const {
  if (!image.empty()) {
    int new_width = static_cast<int>(static_cast<Precision>(image.cols) * new_to_old_size_ratio_);
    int new_height = static_cast<int>(static_cast<Precision>(image.rows) * new_to_old_size_ratio_);

    cv::resize(image, image, cv::Size(new_width, new_height), 0, 0, interpolation_type);
  }
}

void CameraResizer::transformMask(cv::Mat &image) const { transformImage(image, cv::INTER_NEAREST); }

void CameraResizer::transformCalibration(calibration::CameraCalibration &calibration) const {
  calibration.resize(new_to_old_size_ratio_);
}
}  // namespace dsopp::sensors::camera_transformers
