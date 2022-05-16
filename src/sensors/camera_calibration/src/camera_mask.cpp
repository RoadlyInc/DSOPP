#include "sensors/camera_calibration/mask/camera_mask.hpp"

#include <glog/logging.h>
#include <opencv2/imgproc.hpp>

namespace dsopp::sensors::calibration {
CameraMask::CameraMask(const MaskStorage &mask) : mask_(mask.clone()) {
  if (mask_.type() != MaskStorageType) LOG(ERROR) << "Unsupported mask type.";
}

CameraMask::CameraMask(int rows, int cols) : mask_(rows, cols, MaskStorageType, 255) {}

const CameraMask::MaskStorage &CameraMask::data() const { return mask_; }

cv::Mat CameraMask::openCVCompatibleMask() const {
  if (mask_.empty()) return mask_;
  return mask_.clone();
}

CameraMask CameraMask::getEroded(const int border_size) const {
  const int erosion_type = cv::MORPH_RECT;
  cv::Mat element = cv::getStructuringElement(erosion_type, cv::Size(2 * border_size + 1, 2 * border_size + 1),
                                              cv::Point(border_size, border_size));

  cv::Mat dst;

  cv::erode(mask_, dst, element);
  return CameraMask(dst);
}

CameraMask CameraMask::filterSemanticObjects(const cv::Mat &semantics_data,
                                             const semantics::SemanticFilter &semantic_filter) const {
  cv::Mat dst = semantics_data.clone();
  dst.forEach<uint8_t>([&](uint8_t &p, const int *pos) -> void {
    p = semantic_filter.filtered(p) ? 0 : mask_.at<uint8_t>(pos[0], pos[1]);
  });

  return CameraMask(dst);
}

CameraMask CameraMask::resize(const Precision scale) const {
  cv::Mat dst;
  if (mask_.empty()) {
    return CameraMask(mask_);
  }
  cv::resize(mask_, dst, cv::Size(), scale, scale);
  return CameraMask(dst);
}

}  // namespace dsopp::sensors::calibration
