#include "pydsopp/features/extract_tracking_features.hpp"

#include <opencv2/opencv.hpp>

#include "features/camera/sobel_tracking_features_extractor.hpp"
#include "features/camera/tracking_feature.hpp"
#include "features/camera/tracking_features_frame.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"
#include "sensors/camera_calibration/mask/camera_mask.hpp"

namespace dsopp::pydsopp::features {
std::vector<std::pair<dsopp::Precision, dsopp::Precision>> extract_tracking_features(
    const pybind11::array_t<uint8_t> &image_array) {
  const cv::Mat image(static_cast<int>(image_array.shape(0)), static_cast<int>(image_array.shape(1)), CV_8UC3,
                      const_cast<uchar *>(image_array.data()));

  cv::Mat image_gray;
  cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);

  dsopp::sensors::calibration::CameraMask camera_mask(image_gray.rows, image_gray.cols);
  dsopp::features::SobelTrackingFeaturesExtractor tracking_features_extractor;
  const auto tracking_features_frame = tracking_features_extractor.extract(image_gray, camera_mask);

  std::vector<std::pair<dsopp::Precision, dsopp::Precision>> tracking_features_coordinates;
  for (const auto &feature : tracking_features_frame->features()) {
    const auto &coordinates = feature.coordinates();
    tracking_features_coordinates.emplace_back(coordinates.x(), coordinates.y());
  }
  return tracking_features_coordinates;
}
}  // namespace dsopp::pydsopp::features
