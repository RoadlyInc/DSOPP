#ifndef DSOPP_ENERGY_CAMERA_MODELS_DIVISION_MODEL_HPP_
#define DSOPP_ENERGY_CAMERA_MODELS_DIVISION_MODEL_HPP_

#include "common/settings.hpp"
#include "energy/camera_model/camera_model_base.hpp"

#include <Eigen/Dense>

namespace dsopp::energy::model {
/**
 * \brief Division Camera model
 * Main difference from `SimpleRadialCamera` is division by polynomial, not multiplication
 *
 * r_new = r_old / f(r_old)
 *
 * focal, lambda -- parameters
 * @tparam T scalar type
 */
template <typename T = Precision>
class DivisionCameraModel : public CameraModelBase {
 public:
  /** Type of the model */
  static ModelType constexpr Type = ModelType::kDivisionModelCamera;
  /** alias to the base class with template */
  template <typename Scalar>
  using CastT = DivisionCameraModel<Scalar>;
  /** DoF in the intrinsics parameters*/
  static int constexpr DoF = 4;
  static_assert(Model<DivisionCameraModel<T>>);
  /**
   * @param image_size image size
   * @param focal_length focal length
   * @param principal_point principal point
   * @param lambda r^2 coeff
   * @param shutter_time shutter capture time
   * @param scale camera model scale
   */
  DivisionCameraModel(const Eigen::Vector2<Precision> &image_size, T focal_length,
                      const Eigen::Vector2<T> &principal_point, T lambda,
                      time::duration shutter_time = CameraModelBase::kZeroShutterTime, const size_t scale = 1)
      : CameraModelBase(image_size, shutter_time, scale),
        focal_length_(focal_length / T(scale)),
        principal_point_(principal_point / T(scale)),
        lambda_(lambda) {}

  /**
   * @param image_size image size
   * @param intrinsic_parameters intrinsic parameters
   * @param shutter_time shutter capture time
   * @param scale camera model scale
   */
  DivisionCameraModel(const Eigen::Vector2<Precision> &image_size, const Eigen::Vector<T, DoF> &intrinsic_parameters,
                      time::duration shutter_time = CameraModelBase::kZeroShutterTime, const size_t scale = 1)
      : DivisionCameraModel(image_size, intrinsic_parameters(0), intrinsic_parameters.template segment<2>(1),
                            intrinsic_parameters(3), shutter_time, scale) {}

  /**
   * @return intrinsics parameters
   */
  Eigen::Vector<T, DoF> intrinsicsParameters() const {
    return {focal_length_, principal_point_(0), principal_point_(1), lambda_};
  }

  /**
   * projection method
   *
   *        u/f                x
   *        v/f        = alpha y   => 1 + lambda alpha^2 (x^2 + y^2) = alpha z
   *  1 + lambda * r^2         z
   *
   *  => alpha = root(1 - z a + lambda (x^2 + y^2) a^2)
   *
   * @tparam Scalar scalar type
   *
   * @param ray  ray to project to 2d point
   * @param point output value
   * @return bool if proejction was successful
   */
  template <class Scalar>
  bool project(const Eigen::Vector<Scalar, 3> &ray, Eigen::Vector<Scalar, 2> &point) const {
    Scalar lambda_r2 = Scalar(lambda_) * ray.template head<2>().squaredNorm();
    Scalar z = ray(2);
    Scalar alpha = (z - std::sqrt(z * z - Scalar(4.0) * lambda_r2)) / Scalar(2.0) / lambda_r2;

    point = alpha * ray.template head<2>() * Scalar(focal_length_) + principal_point_.template cast<Scalar>();
    return insideCameraROI(point);
  }
  /**
   * unprojection method
   *
   * @tparam Scalar scalar type
   * @param point 2d point to project to ray
   * @param ray output value
   * @return bool if proejction was successful
   */
  template <class EigenDerived, class Scalar>
  bool unproject(const Eigen::MatrixBase<EigenDerived> &point, Eigen::Vector<Scalar, 3> &ray) const {
    if (!insideCameraROI(point)) return false;
    ray.template head<2>() = (point - principal_point_.template cast<Scalar>()) / Scalar(focal_length_);
    Scalar r2 = ray.template head<2>().squaredNorm();
    ray(2) = Scalar(1.0) + r2 * Scalar(lambda_);
    return true;
  }
  /**
   * @return focal length in X axis
   */
  T focalX() const { return focal_length_; }
  /**
   * @return focal length in Y axis
   */
  T focalY() const { return focal_length_; }

 private:
  /** focal length */
  const T focal_length_;
  /**principal point */
  Eigen::Vector2<T> principal_point_;
  /** r^2 coeff */
  const T lambda_;
};

}  // namespace dsopp::energy::model

#endif  // DSOPP_ENERGY_CAMERA_MODELS_DIVISION_MODEL_HPP_
