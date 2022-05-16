#ifndef DSOPP_SRC_SENSORS_CAMERA_TRANSFORMERS_CAMERA_IMAGE_CROP_HPP_
#define DSOPP_SRC_SENSORS_CAMERA_TRANSFORMERS_CAMERA_IMAGE_CROP_HPP_

#include <Eigen/Dense>

namespace dsopp::sensors::camera_transformers {

/**
 * Function to perform image size crop in order to ease pyramid creation
 * @tparam numberOfPyramids number of pyramid levels
 * @tparam Scalar
 * @param initial_size initial size
 * @return output size which is divided by 2^numberOfPyramids
 */
template <size_t numberOfPyramids, class Scalar>
Eigen::Vector2<Scalar> cropSizePowerOf2(const Eigen::Vector2<Scalar> &initial_size) {
  size_t width = (static_cast<size_t>(initial_size(0)) >> numberOfPyramids) * (1 << numberOfPyramids);
  size_t height = (static_cast<size_t>(initial_size(1)) >> numberOfPyramids) * (1 << numberOfPyramids);
  return Eigen::Vector2<Scalar>(static_cast<Scalar>(width), static_cast<Scalar>(height));
}

}  // namespace dsopp::sensors::camera_transformers
#endif  // DSOPP_SRC_SENSORS_CAMERA_TRANSFORMERS_CAMERA_IMAGE_CROP_HPP_
