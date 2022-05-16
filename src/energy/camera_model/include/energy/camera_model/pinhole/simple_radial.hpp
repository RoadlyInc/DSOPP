#ifndef DSOPP_PINHOLE_SIMPLE_RADIAL_HPP
#define DSOPP_PINHOLE_SIMPLE_RADIAL_HPP

#include <ceres/jet.h>
#include <Eigen/Dense>

#include "common/settings.hpp"
#include "energy/camera_model/camera_model_base.hpp"
#include "energy/camera_model/polynomial_solver.hpp"

namespace dsopp::energy::model {

/**
 * \brief Simple radial camera model
 *
 * f, cx, cy, k1, k2 -- parameters
 */
template <typename T = Precision>
class SimpleRadialCamera : public CameraModelBase {
 public:
  /** Type of the model */
  static ModelType constexpr Type = ModelType::kSimpleRadialCamera;
  /** alias to the base class with template */
  template <typename Scalar>
  using CastT = SimpleRadialCamera<Scalar>;
  /** DoF in the intrinsics parameters*/
  static int constexpr DoF = 5;
  static_assert(Model<SimpleRadialCamera<T>>);
  /**
   * Constructor
   *
   * @param image_size image size
   * @param focal_length focal length
   * @param principal_point principal point
   * @param k1 2nd degree coefficient in distortion polynomial
   * @param k2 4nd degree coefficient in distortion polynomial
   * @param shutter_time shutter capture time
   * @param scale camera model scale
   */
  SimpleRadialCamera(const Eigen::Vector2<Precision> &image_size, T focal_length,
                     const Eigen::Vector2<T> &principal_point, T k1, T k2,
                     time::duration shutter_time = CameraModelBase::kZeroShutterTime, const size_t scale = 1)
      : CameraModelBase(image_size, shutter_time, scale),
        focal_length_(focal_length / T(int(scale))),
        principal_point_(principal_point / T(int(scale))),
        k1_(k1),
        k2_(k2) {
    T max_x = std::max(principal_point_(0), T(image_size_(0)) - principal_point_(0));
    T max_y = std::max(principal_point_(1), T(image_size_(1)) - principal_point_(1));

    max_valid_radius_ = ceres::sqrt(max_x * max_x + max_y * max_y) / focal_length_;

    Eigen::VectorX<T> polynomial(6);
    polynomial << -max_valid_radius_, T(1), T(0), k1, T(0), k2;
    max_valid_radius_ = polynomial_solver::minimalPositiveRoot(polynomial);

    /**
     * df/dr = 1 + 3 k1 r^2 + 5 k2 r^4
     * we need minimum positive root
     *
     * b^2 - 4 ac
     */
    T discriminant = T(9) * k1 * k1 - T(20) * k2;
    if (discriminant >= T(0)) {
      T root1_2 = (T(-3) * k1 - ceres::sqrt(discriminant)) / T(10) / k2;
      T root2_2 = (T(-3) * k1 + ceres::sqrt(discriminant)) / T(10) / k2;
      T root = max_valid_radius_;

      if (root1_2 > T(0)) {
        root = ceres::sqrt(root1_2);
      } else if (root2_2 > T(0)) {
        root = ceres::sqrt(root2_2);
      }

      if (root < max_valid_radius_) {
        max_valid_radius_ = root - T(kBorderSize) / focal_length_;
      }
    }

    T r2 = max_valid_radius_ * max_valid_radius_;
    T r4 = r2 * r2;
    line_after_radius_(1) = max_valid_radius_ * (T(1) + k1 * r2 + k2 * r4);
    line_after_radius_(0) = (T(1) + T(3) * k1 * r2 + T(5) * k2 * r4);
  }

  /**
   * Constructor
   *
   * @param image_size image size
   * @param intrinsic_parameters focal length, coordinates of principal point in pixels and distortion coefficients
   * @param shutter_time shutter capture time
   * @param scale camera model scale
   */
  SimpleRadialCamera(const Eigen::Vector2<Precision> &image_size, const Eigen::Vector<T, DoF> &intrinsic_parameters,
                     time::duration shutter_time = CameraModelBase::kZeroShutterTime, const size_t scale = 1)
      : SimpleRadialCamera(image_size, intrinsic_parameters(0), intrinsic_parameters.template segment<2>(1),
                           intrinsic_parameters(3), intrinsic_parameters(4), shutter_time, scale) {}

  /**
   * @return focal length in X axis
   */
  T focalX() const { return focal_length_; }
  /**
   * @return focal length in Y axis
   */
  T focalY() const { return focal_length_; }

  /** transform intrinsic parameters to the string
   *
   * @return string with intrinsic parameters
   */
  std::string toString() const {
    return "simple_radial\n" + std::to_string(this->image_size()(0)) + " " + std::to_string(this->image_size()(1)) +
           "\n" + std::to_string(focal_length_) + "\n" + std::to_string(principal_point_(0)) + " " +
           std::to_string(principal_point_(1)) + "\n" + std::to_string(k1_) + " " + std::to_string(k2_);
  }

  /**
   * @return intrinsics parameters
   */
  Eigen::Vector<T, DoF> intrinsicsParameters() const {
    return {focal_length_, principal_point_(0), principal_point_(1), k1_, k2_};
  }

  /**
   * projection method
   *
   * @tparam Scalar scalar type
   *
   * @param ray_  ray to project to 2d point
   * @param point output value
   * @return bool if proejction was successful
   */
  template <class Scalar>
  bool project(const Eigen::Vector<Scalar, 3> &ray_, Eigen::Vector<Scalar, 2> &point) const {
    bool success = true;
    Eigen::Vector<Scalar, 2> ray = ray_.hnormalized();

    Scalar r2 = ray.squaredNorm();
    Scalar r_delta;

    /**
     * new radius function is:
     * r_new = r_old + k1 r_old^3 + k2 r_old_5
     *
     * out of image boundraies it may be decresing, so rays with large angle with principal ray
     * may appear inside image
     * in order to prevent it r_new function is approximated with line outside of image
     */
    if (sqrt(r2) > Scalar(max_valid_radius_)) {
      r_delta = Scalar(line_after_radius_(0)) + Scalar(line_after_radius_(1)) / ceres::sqrt(r2);
      success = false;
    } else {
      r_delta = Scalar(1) + Scalar(k1_) * r2 + Scalar(k2_) * r2 * r2;
    }

    ray *= r_delta;

    point = ray * Scalar(focal_length_) + principal_point_.template cast<Scalar>();
    return success && insideCameraROI(point);
  }

  /**
   * projection method
   *
   * @tparam Scalar scalar type
   *
   * @param ray_  ray to project to 2d point
   * @param[out] point output value
   * @param[out] jacobian jacobian
   * @return bool if proejction was successful
   */
  template <class Scalar>
  bool project(const Eigen::Vector<Scalar, 3> &ray_, Eigen::Vector<Scalar, 2> &point,
               Eigen::Matrix<Scalar, 2, 3> &jacobian) const {
    bool success = true;
    Eigen::Vector<Scalar, 2> ray = ray_.hnormalized();

    Scalar r2 = ray.squaredNorm();

    /** when we are projection fraom invalid distortion range */
    if (sqrt(r2) > Scalar(max_valid_radius_)) {
      return false;
    }
    Scalar r_delta = Scalar(1) + Scalar(k1_) * r2 + Scalar(k2_) * r2 * r2;

    ray *= r_delta;

    point = ray * Scalar(focal_length_) + principal_point_.template cast<Scalar>();

    Scalar xy_common = (Scalar(2 * k1_) + Scalar(4 * k2_) * r2) / ray_(2) / ray_(2);

    jacobian(0, 0) = Scalar(focal_length_) * (ray_(0) * ray_(0) * xy_common + r_delta) / ray_(2);
    jacobian(0, 1) = Scalar(focal_length_) * ray_(0) * ray_(1) * xy_common / ray_(2);

    jacobian(1, 0) = jacobian(0, 1);
    jacobian(1, 1) = Scalar(focal_length_) * (ray_(1) * ray_(1) * xy_common + r_delta) / ray_(2);

    jacobian.template block<2, 1>(0, 2) = Scalar(focal_length_) * ray_.template head<2>() / ray_(2) / ray_(2);
    jacobian.template block<2, 1>(0, 2) *= (-Scalar(3 * k1_) * r2 - Scalar(5 * k2_) * r2 * r2 - Scalar(1));

    return success && insideCameraROI(point);
  }
  /**
   * unprojection method
   *
   * @tparam Scalar scalar type
   * @param point_ 2d point to project to ray
   * @param ray output value
   * @return bool if proejction was successful
   */
  template <class EigenDerived, class Scalar>
  bool unproject(const Eigen::MatrixBase<EigenDerived> &point_, Eigen::Vector<Scalar, 3> &ray) const {
    Eigen::Vector<Scalar, 2> point = (point_ - principal_point_.template cast<Scalar>()) / Scalar(focal_length_);

    Eigen::VectorX<Scalar> polynomial(6);
    polynomial(0) = -point.norm();
    polynomial(1) = Scalar(1);
    polynomial(2) = Scalar(0);
    polynomial(3) = Scalar(k1_);
    polynomial(4) = Scalar(0);
    polynomial(5) = Scalar(k2_);

    Scalar new_r = polynomial_solver::minimalPositiveRoot(polynomial);
    ray = (new_r * point.normalized()).homogeneous();

    return (insideCameraROI(point_));
  }

  /**
   * get subset of parameter's indices which are constant
   * @param fix_focal fix focal length parameters
   * @param fix_center fix center parameters
   * @param fix_model_specific fix specific parameters (in case of pinhole useless option)
   * @return vector of constant parameter's indices
   */
  static std::vector<int> constantParameterSet(const bool fix_focal, const bool fix_center,
                                               const bool fix_model_specific) {
    std::vector<int> constant_parameters;
    if (fix_focal) constant_parameters = {0};
    if (fix_center) {
      constant_parameters.push_back(1);
      constant_parameters.push_back(2);
    }
    if (fix_model_specific) {
      constant_parameters.push_back(3);
      constant_parameters.push_back(4);
    }
    return constant_parameters;
  }

  /** get principal point
   *
   * @return principal point
   */
  const Eigen::Vector2<T> &principal_point() const { return principal_point_; }

 private:
  /** focal length */
  const T focal_length_;
  /** principal point */
  const Eigen::Vector2<T> principal_point_;
  /** 2nd degree distortion coefficient */
  const T k1_;
  /** 4nd degree distortion coefficient */
  const T k2_;

  /** maximum valid radius for projecting ray onto image */
  T max_valid_radius_;
  /** linear extropalation of distortion out of image bound */
  Eigen::Vector2<T> line_after_radius_;
};
}  // namespace dsopp::energy::model
#endif  // DSOPP_PINHOLE_SIMPLE_RADIAL_HPP
