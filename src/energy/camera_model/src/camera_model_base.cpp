#include "energy/camera_model/camera_model_base.hpp"

namespace dsopp::energy::model {
CameraModelBase::CameraModelBase(const Eigen::Vector2<Precision> &image_size, time::duration shutter_time,
                                 const size_t scale)
    : image_size_(image_size / scale), shutter_time_(shutter_time / scale) {}

const Eigen::Vector2<Precision> &CameraModelBase::image_size() const { return image_size_; }

time::duration CameraModelBase::shutterTime() const { return shutter_time_; }

}  // namespace dsopp::energy::model
