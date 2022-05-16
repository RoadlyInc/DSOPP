#include "energy/camera_model/pinhole/ios_camera_model.hpp"

namespace dsopp::energy::model {

IOSCamera::IOSCamera(const Eigen::Vector2<Precision> &image_size, const Eigen::Vector2<Precision> &focal_lengths,
                     const Eigen::Vector2<Precision> &center, const Eigen::VectorX<Precision> &lookup_distortion,
                     time::duration shutter_time)
    : CameraModelBase(image_size, shutter_time) {
  long N = focal_lengths.rows() + center.rows() + lookup_distortion.rows();
  data_.resize(N);
  data_.head<2>() = focal_lengths;
  data_.segment<2>(2) = center;
  data_.tail(lookup_distortion.rows()) = lookup_distortion;

  /** estimating max radius */
  Precision max_x = std::max(center(0), image_size(0) - center(0));
  Precision max_y = std::max(center(1), image_size(1) - center(1));

  max_radius_ = std::sqrt(max_x * max_x + max_y * max_y);
}

Precision IOSCamera::focalX() const { return data_(0); }
Precision IOSCamera::focalY() const { return data_(1); }

}  // namespace dsopp::energy::model
