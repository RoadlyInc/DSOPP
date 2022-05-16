#ifndef DSOPP_SENSORS_CAMERA_TRANSFORMERS_CAMERA_TRANSFORMER_HPP_
#define DSOPP_SENSORS_CAMERA_TRANSFORMERS_CAMERA_TRANSFORMER_HPP_

#include <memory>

#include <opencv2/opencv.hpp>

#include "sensors/camera_calibration/camera_calibration.hpp"

namespace dsopp::sensors::camera_transformers {

/**
 * \brief Base class for camera transformers
 *
 */
class CameraTransformer {
 public:
  /**
   * transform image
   * @param[out] image input image
   * @param interpolation_type OpenCV interpolation type
   */
  virtual void transformImage(cv::Mat &image, int interpolation_type = cv::INTER_LINEAR) const = 0;
  /**
   * transform mask (with nearest neigh interpolation)
   * @param[out] image input image
   */
  virtual void transformMask(cv::Mat &image) const = 0;
  /**
   * transform calibration
   * @param[out] calibration camera calibration
   */
  virtual void transformCalibration(calibration::CameraCalibration &calibration) const = 0;
  virtual ~CameraTransformer() = default;
};

cv::Mat runImageTransformers(const std::vector<std::unique_ptr<camera_transformers::CameraTransformer>> &transformers,
                             const cv::Mat &image);

cv::Mat runMaskTransformers(const std::vector<std::unique_ptr<camera_transformers::CameraTransformer>> &transformers,
                            const cv::Mat &mask);

}  // namespace dsopp::sensors::camera_transformers

#endif  // DSOPP_SENSORS_CAMERA_TRANSFORMERS_CAMERA_TRANSFORMER_HPP_
