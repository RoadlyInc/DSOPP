#include "features/camera/photometrically_corrected_image.hpp"

#include <glog/logging.h>
#include <Eigen/Dense>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>

namespace dsopp::features {
std::vector<Precision> photometricallyCorrectedImage(const cv::Mat &image,
                                                     const std::array<Precision, 256> &photometric_calibration,
                                                     const cv::Mat &vignetting) {
  double max_value_vignetting;
  if (!vignetting.empty()) {
    cv::minMaxLoc(vignetting, nullptr, &max_value_vignetting);
  }
  CHECK(vignetting.empty() || image.size() == vignetting.size());

  std::vector<Precision> result(static_cast<size_t>(image.rows * image.cols));
  auto width = static_cast<long>(image.cols);
  auto height = static_cast<long>(image.rows);
  Eigen::Map<Eigen::Array<Precision, -1, -1, Eigen::RowMajor>> result_map(result.data(), height, width);
  using MAPOpenCV = Eigen::Map<const Eigen::Array<uchar, -1, -1, Eigen::RowMajor>>;
  MAPOpenCV map_image(image.data, height, width);
  MAPOpenCV map_vignetting(vignetting.data, height, width);
  result_map = map_image.unaryExpr([&](uchar x) { return photometric_calibration[static_cast<size_t>(x)]; });
  if (!vignetting.empty()) {
    result_map *= max_value_vignetting / (map_vignetting.cast<Precision>() + 1);
  }
  return result;
}
}  // namespace dsopp::features
