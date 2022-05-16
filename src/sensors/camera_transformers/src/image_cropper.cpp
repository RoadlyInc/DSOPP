#include "sensors/camera_transformers/image_cropper.hpp"

#include "sensors/camera_transformers/camera_image_crop.hpp"

namespace dsopp::sensors::camera_transformers {

void ImageCropper::transformImage(cv::Mat &image, int) const {
  if (!image.empty()) {
    Eigen::Vector2i cropped_new_image_size =
        cropSizePowerOf2<calibration::CameraCalibration::kNumberOfPyramidLevels, int>(
            Eigen::Vector2i(image.cols, image.rows));
    image = image(cv::Range(0, cropped_new_image_size[1]), cv::Range(0, cropped_new_image_size[0]));
  }
}

void ImageCropper::transformMask(cv::Mat &image) const { transformImage(image); }

void ImageCropper::transformCalibration(calibration::CameraCalibration &calibration) const { calibration.crop(); }
}  // namespace dsopp::sensors::camera_transformers
