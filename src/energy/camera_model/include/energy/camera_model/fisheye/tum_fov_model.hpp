#ifndef DSOPP_ENERGY_CAMERA_MODEL_TUM_FOV_MODEL_HPP_
#define DSOPP_ENERGY_CAMERA_MODEL_TUM_FOV_MODEL_HPP_

#include <Eigen/Dense>

#include "common/settings.hpp"
#include "energy/camera_model/camera_model_base.hpp"

namespace dsopp::energy::model {

/**
 * \brief FOV camera model
 *
 * Model from TUM-mono sequence
 * ref: https://github.com/tum-vision/mono_dataset_code/blob/master/src/FOVUndistorter.cpp
 */
template <typename T = Precision>
class TUMFovModel : public CameraModelBase {
 public:
  /** Type of the model */
  static ModelType constexpr Type = ModelType::kTumFovCamera;
  /** alias to the base class with template */
  template <typename Scalar>
  using CastT = TUMFovModel<Scalar>;
  /** DoF in the intrinsics parameters*/
  static int constexpr DoF = 5;
  static_assert(Model<TUMFovModel<T>>);

  /**
   * @param image_size image size
   * @param focal_lengths focal length
   * @param principal_point principal point
   * @param fov field of view
   * @param shutter_time shutter capture time
   * @param scale camera model scale
   */
  TUMFovModel(const Eigen::Vector2<Precision> &image_size, const Eigen::Vector2<T> &focal_lengths,
              const Eigen::Vector2<T> &principal_point, const T &fov,
              time::duration shutter_time = CameraModelBase::kZeroShutterTime, const size_t scale = 1)
      : CameraModelBase(image_size, shutter_time, scale),
        focal_lengths_(focal_lengths / T(int(scale))),
        principal_point_(principal_point / T(int(scale))),
        fov_(fov) {}

  /**
   * @return focal length in X axis
   */
  T focalX() const { return focal_lengths_[0]; }
  /**
   * @return focal length in Y axis
   */
  T focalY() const { return focal_lengths_[1]; }
  /** get principal point
   *
   * @return principal point
   */
  const Eigen::Vector2<T> &principal_point() const { return principal_point_; }

  /** Default destructor */
  ~TUMFovModel() = default;

  /**
   * projection method
   *
   * @tparam Scalar scalar type
   *
   * @param ray_  ray to project to 2d point
   * @param point output value
   * @return bool if projection was successful
   */
  template <class Scalar>
  bool project(const Eigen::Vector<Scalar, 3> &ray_, Eigen::Vector<Scalar, 2> &point) const {
    Scalar r_u = ray_.template head<2>().norm();
    if (r_u < Scalar(1e-8)) {
      point = principal_point_.template cast<Scalar>();
      return true;
    }
    Scalar r_d = atan2(2 * r_u * tan(fov_ / 2), ray_(2)) / fov_;
    point = r_d / r_u * ray_.template head<2>().array() * focal_lengths_.array();
    point = point + principal_point_;
    return insideCameraROI(point);
  }

  /**
   * unprojection method
   *
   * @tparam Scalar scalar type
   * @param point_ 2d point to project to ray
   * @param ray output value
   * @return bool if projection was successful
   */
  template <class EigenDerived, class Scalar>
  bool unproject(const Eigen::MatrixBase<EigenDerived> &point_, Eigen::Vector<Scalar, 3> &ray) const {
    Eigen::Vector2<Scalar> point = point_ - principal_point_;
    point = point.array() / focal_lengths_.array();

    Scalar r_d = point.norm();
    if (r_d < Scalar(1e-8)) {
      ray = Eigen::Vector3<Precision>(0, 0, 1).cast<Scalar>();
      return true;
    }

    ray(2) = 1 / tan(r_d * fov_);
    ray.template head<2>() = point / (2 * r_d * tan(fov_ / 2));
    return insideCameraROI(point_);
  }

 private:
  /** focal length */
  Eigen::Vector2<T> focal_lengths_;
  /** principal point */
  Eigen::Vector2<T> principal_point_;
  /** field of view */
  T fov_;
};

}  // namespace dsopp::energy::model

#endif  // DSOPP_ENERGY_CAMERA_MODEL_TUM_FOV_MODEL_HPP_
