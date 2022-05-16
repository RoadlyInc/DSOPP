#include "energy/camera_model/fisheye/atan_camera.hpp"

namespace dsopp::energy::model {

AtanCamera::AtanCamera(const Eigen::Vector2<Precision> &image_size, const Eigen::Vector2<Precision> &focal_lengths,
                       const Eigen::Vector2<Precision> &center, const Eigen::VectorX<Precision> &polynomial,
                       time::duration shutter_time)
    : CameraModelBase(image_size, shutter_time) {
  long N = focal_lengths.rows() + center.rows() + polynomial.rows();
  data_.resize(N);
  data_.head<2>() = focal_lengths;
  data_.segment<2>(2) = center;
  data_.tail(polynomial.rows()) = polynomial;
}

Precision AtanCamera::focalX() const { return data_(0); }
Precision AtanCamera::focalY() const { return data_(1); }

}  // namespace dsopp::energy::model
