#ifndef DSOPP_SRC_SENSORS_CAMERA_TRANSFORMERS_IMAGE_CROPPER_HPP_
#define DSOPP_SRC_SENSORS_CAMERA_TRANSFORMERS_IMAGE_CROPPER_HPP_

#include "sensors/camera_transformers/camera_transformer.hpp"

#include "common/settings.hpp"

namespace dsopp::sensors::camera_transformers {

/**
 * \brief Perform image size crop in order to ease pyramid creation
 *
 */
class ImageCropper : public CameraTransformer {
 public:
  void transformImage(cv::Mat &input, int interpolation_type = cv::INTER_LINEAR) const override;
  void transformMask(cv::Mat &input) const override;
  void transformCalibration(calibration::CameraCalibration &calibration) const override;
  ~ImageCropper() = default;
};

}  // namespace dsopp::sensors::camera_transformers

#endif  // DSOPP_SRC_SENSORS_CAMERA_TRANSFORMERS_IMAGE_CROPPER_HPP_
