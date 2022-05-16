#include "sensors/camera_calibration/camera_calibration.hpp"

#include <glog/logging.h>

#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/camera_model/pinhole/simple_radial.hpp"
#include "sensors/camera_transformers/camera_image_crop.hpp"

namespace dsopp::sensors::calibration {

CameraCalibration::CameraCalibration(const Eigen::Vector2<Precision> &image_size,
                                     const Eigen::VectorX<Precision> &camera_intrinsics, energy::model::ModelType type,
                                     const time::duration shutter_time, Undistorter &&undistorter)
    : image_size_(image_size),
      camera_intrinsics_(camera_intrinsics),
      type_(type),
      shutter_time_(shutter_time),
      undistorter_(std::move(undistorter)) {}

CameraCalibration::CameraCalibration(const Eigen::Vector2<Precision> &image_size,
                                     const Eigen::VectorX<Precision> &camera_intrinsics, energy::model::ModelType type)
    : CameraCalibration(image_size, camera_intrinsics, type, std::chrono::seconds(0),
                        Undistorter::Identity(image_size)) {}

CameraCalibration::CameraCalibration(CameraCalibration &&other) = default;

const Eigen::Vector2<Precision> &CameraCalibration::image_size() const { return image_size_; }

const Eigen::VectorX<Precision> &CameraCalibration::cameraIntrinsics() const { return camera_intrinsics_; }

energy::model::ModelType CameraCalibration::type() const { return type_; }

void CameraCalibration::resize(const Precision &new_size_old_size_ratio) {
  image_size_ *= new_size_old_size_ratio;
  if (type_ == energy::model::ModelType::kPinholeCamera) {
    camera_intrinsics_ *= new_size_old_size_ratio;
  } else if (type_ == energy::model::ModelType::kSimpleRadialCamera) {
    camera_intrinsics_.head<3>() *= new_size_old_size_ratio;
  } else {
    LOG(ERROR) << "Unknown type of calibration for resize";
  }
}

void CameraCalibration::crop() {
  image_size_ = camera_transformers::cropSizePowerOf2<kNumberOfPyramidLevels, Precision>(image_size_);
}

std::optional<Precision> CameraCalibration::focal() const {
  switch (type()) {
    case energy::model::ModelType::kPinholeCamera: {
      return cameraModel<energy::model::PinholeCamera<Precision>>()->focalX();
      break;
    }
    case energy::model::ModelType::kSimpleRadialCamera: {
      return cameraModel<energy::model::SimpleRadialCamera<Precision>>()->focalX();
      break;
    }
    default:
      LOG(WARNING) << "Unsupported camera type in the track" << static_cast<size_t>(type());
      return std::nullopt;
  }
}

time::duration CameraCalibration::shutterTime() const { return shutter_time_; }

template <energy::model::Model Model>
std::unique_ptr<Model> CameraCalibration::cameraModel(size_t level_shift) const {
  CHECK(Model::Type == type_ && camera_intrinsics_.size() == Model::DoF);
  return std::make_unique<Model>(image_size_, camera_intrinsics_, shutter_time_, 1 << level_shift);
}

const Undistorter &CameraCalibration::undistorter() const { return undistorter_; }

void CameraCalibration::setParameters(const Eigen::VectorX<Precision> &camera_intrinsics) {
  CHECK(camera_intrinsics.size() == camera_intrinsics_.size());
  camera_intrinsics_ = camera_intrinsics;
}
CameraCalibration::CameraCalibration(const CameraCalibration &other) = default;
CameraCalibration CameraCalibration::clone() const { return *this; }

template std::unique_ptr<energy::model::PinholeCamera<Precision>> CameraCalibration::cameraModel(size_t level) const;
template std::unique_ptr<energy::model::SimpleRadialCamera<Precision>> CameraCalibration::cameraModel(
    size_t level) const;

}  // namespace dsopp::sensors::calibration
