
#ifndef DSOPP_CAMERA_CALIBRATION_HPP
#define DSOPP_CAMERA_CALIBRATION_HPP

#include <Eigen/Dense>
#include <memory>
#include <optional>

#include "common/settings.hpp"
#include "common/time/time.hpp"

#include "energy/camera_model/camera_model_base.hpp"
#include "sensors/camera_calibration/undistorter/undistorter.hpp"

namespace dsopp {

namespace sensors {
namespace calibration {
/**
 * \brief Camera Calibration
 *
 * stores image size, camera intrinsics and camera shutter capture time. Can creates model
 */
class CameraCalibration {
 public:
  /** number of levels in the pyramids */
  static constexpr size_t kNumberOfPyramidLevels = 4;
  /**
   * @param image_size image size
   * @param camera_intrinsics parameters of the camera model (for ex. focal_lengths and principal point for the pinhole
   * model)
   * @param type type of the camera model (pinhole, simple radial, etc.)
   * @param shutter_time shutter capture time
   * @param undistorter remap distorted image to undistorted
   */
  CameraCalibration(const Eigen::Vector2<Precision> &image_size, const Eigen::VectorX<Precision> &camera_intrinsics,
                    energy::model::ModelType type, const time::duration shutter_time, Undistorter &&undistorter);
  /**
   * @param image_size image size
   * @param camera_intrinsics parameters of the camera model (for ex. focal_lengths and principal point for the pinhole
   * model)
   * @param type type of the camera model (pinhole, simple radial, etc.)
   */
  CameraCalibration(const Eigen::Vector2<Precision> &image_size, const Eigen::VectorX<Precision> &camera_intrinsics,
                    energy::model::ModelType type);
  /**
   * move constructor
   */
  CameraCalibration(CameraCalibration &&);

  /** Method to get image size
   *
   * @return image size
   */
  const Eigen::Vector2<Precision> &image_size() const;
  /** Method to get camera intrinsics
   *
   * @return camera intrinsics
   */
  const Eigen::VectorX<Precision> &cameraIntrinsics() const;
  /**
   * @return type of the camera model (pinhole, simple radial, etc.)
   */
  energy::model::ModelType type() const;
  /**
   * resize calibration object
   * @param new_size_old_size_ratio ration new_size / old_size
   */
  void resize(const Precision &new_size_old_size_ratio);

  /**
   * crop image size
   */
  void crop();

  /**
   * @return shutter time
   */
  time::duration shutterTime() const;

  /**
   * method create CameraModel from the intrinsic parameters
   *
   * @param level_shift bit shift by pyramid level
   * @return CameraModel object
   */
  template <energy::model::Model Model>
  std::unique_ptr<Model> cameraModel(size_t level_shift = 0) const;
  /** Method to get undistorter pointer
   *
   * @return undistorter const reference
   */
  const Undistorter &undistorter() const;

  /**
   * Return focal length of the camera model
   *
   * @return focal length
   */
  std::optional<Precision> focal() const;

  /**
   * method to set new intrinsics parameters
   * @param camera_intrinsics camera parameters
   */
  void setParameters(const Eigen::VectorX<Precision> &camera_intrinsics);
  /**
   * @return deep copy of the object
   */
  CameraCalibration clone() const;

 protected:
  /**
   * creates deep copy of the object
   */
  CameraCalibration(const CameraCalibration &);

 private:
  /** Width and height of the image*/
  Eigen::Vector2<Precision> image_size_;
  /** stores camera calibration for producing camera model.*/
  Eigen::VectorX<Precision> camera_intrinsics_;
  /** type of the camera model (pinhole, simple radial, etc.) */
  energy::model::ModelType type_;
  /** shutter time */
  const time::duration shutter_time_;
  /** undistorter pointer  */
  Undistorter undistorter_;
};

}  // namespace calibration
}  // namespace sensors
}  // namespace dsopp
#endif  // DSOPP_CAMERA_CALIBRATION_HPP
