#ifndef SENSORS_CAMERA_TRANSFORMERS_CAMERA_RESIZER_HPP_
#define SENSORS_CAMERA_TRANSFORMERS_CAMERA_RESIZER_HPP_

#include "sensors/camera_transformers/camera_transformer.hpp"

#include "common/settings.hpp"

namespace dsopp::sensors::camera_transformers {

/**
 * \brief Change image size to ``new_to_old_size_ratio_`` * ``image_size``
 *
 */
class CameraResizer : public CameraTransformer {
 public:
  /**
   * @param new_to_old_size_ratio ratio of ``new_size / old_size``
   */
  CameraResizer(const Precision &new_to_old_size_ratio);
  void transformImage(cv::Mat &input, int interpolation_type = cv::INTER_LINEAR) const override;
  void transformMask(cv::Mat &input) const override;
  void transformCalibration(calibration::CameraCalibration &calibration) const override;
  ~CameraResizer() = default;

 private:
  /** ratio of ``new_size / old_size`` */
  const Precision new_to_old_size_ratio_;
};

}  // namespace dsopp::sensors::camera_transformers

#endif  // SENSORS_CAMERA_TRANSFORMERS_CAMERA_RESIZER_HPP_
