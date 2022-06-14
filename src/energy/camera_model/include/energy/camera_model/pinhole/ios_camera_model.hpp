#ifndef DSOPP_ENERGY_CAMERA_MODEL_PINHOLE_IOS_CAMERA_MODEL_HPP
#define DSOPP_ENERGY_CAMERA_MODEL_PINHOLE_IOS_CAMERA_MODEL_HPP

#include "common/settings.hpp"
#include "energy/camera_model/camera_model_base.hpp"

#include <Eigen/Dense>

namespace dsopp::energy::model {

/**
 * \brief Camera model for ios device
 *
 * Model with lookup table for point undistortion
 */
class IOSCamera : public CameraModelBase {
  /** alias for templated Eigen vector */
  template <typename T, int N>
  using Vec = Eigen::Matrix<T, N, 1>;

 public:
  ~IOSCamera() = default;

  /**
   * @return focal length in X axis
   */
  Precision focalX() const;
  /**
   * @return focal length in Y axis
   */
  Precision focalY() const;

  /**
   * creates IOS camera model
   *
   * @param image_size image size
   * @param focal_lengths focal lengths
   * @param center projection center
   * @param lookup_distortion lookup table to correct radius
   * @param shutter_time shutter capture time
   */
  IOSCamera(const Eigen::Vector2<Precision> &image_size, const Eigen::Vector2<Precision> &focal_lengths,
            const Eigen::Vector2<Precision> &center, const Eigen::VectorX<Precision> &lookup_distortion,
            time::duration shutter_time = CameraModelBase::kZeroShutterTime);

  /**
   * projection templated method
   *
   * @param ray_  ray to project to 2d point
   * @param point output value
   * @return bool if proejction was successful
   */
  template <typename Scalar>
  bool project(const Vec<Scalar, 3> &ray_, Vec<Scalar, 2> &point) const {
    return project_internal(ray_, point);
  }

  /**
   * unprojection templated method
   *
   * @param point_ 2d point to project to ray
   * @param ray output value
   * @return bool if proejction was successful
   */
  template <typename Scalar>
  bool unproject(const Vec<Scalar, 2> &point_, Vec<Scalar, 3> &ray) const {
    if (!insideCameraROI(point_)) return false;

    Vec<Scalar, 2> point = point_ - data_.template segment<2>(2).cast<Scalar>();

    Scalar r_ratio = point.norm() / static_cast<Scalar>(max_radius_);
    point *= undistortion_magnifier(r_ratio);

    point = point.array() / data_.template head<2>().cast<Scalar>().array();

    ray = point.homogeneous();

    /** some Gauss–Newton iterations to refine answer */
    Eigen::Matrix<Scalar, 2, 3> J;

    const int kGaussNewtoniteration_number = 7;

    for (int i = 0; i < kGaussNewtoniteration_number; ++i) {
      Vec<Scalar, 2> pt_2d;
      if (!project_internal(ray, pt_2d, &J)) return false;
      Vec<Scalar, 2> iterationResidual = point_ - pt_2d;
      Vec<Scalar, 3> delta_ray = (J.transpose() * J).colPivHouseholderQr().solve(J.transpose() * iterationResidual);
      ray += delta_ray;
    }
    ray /= ray(2);
    return true;
  }

 private:
  /**
   * projection method
   *
   * @param ray_  ray to project to 2d point
   * @param point output value
   * @param J jacobian of projection, evaluated if needed (default nullptr)
   * @return bool if proejction was successful
   */
  template <typename Scalar>
  bool project_internal(const Vec<Scalar, 3> &ray_, Vec<Scalar, 2> &point,
                        Eigen::Matrix<Scalar, 2, 3> *J = nullptr) const {
    if (abs(ray_(2)) < Scalar(1e-12)) return false;

    Vec<Scalar, 2> ray = ray_.hnormalized();

    ray = ray.array() * data_.template head<2>().cast<Scalar>().array();

    Scalar r_ratio = ray.norm() / static_cast<Scalar>(max_radius_);
    Scalar magnifier = distortion_magnifier(r_ratio);

    ray /= magnifier;
    point = ray + data_.template segment<2>(2).cast<Scalar>();

    if (J) {
      Scalar z2 = ray_(2) * ray_(2);
      J->setZero();
      (*J)(0, 0) = magnifier * Scalar(data_(0)) / ray_(2);
      (*J)(1, 1) = magnifier * Scalar(data_(1)) / ray_(2);
      (*J)(0, 2) = -magnifier * Scalar(data_(0)) * ray_(0) / z2;
      (*J)(1, 2) = -magnifier * Scalar(data_(1)) * ray_(1) / z2;
    }
    return insideCameraROI(point);
  }

  /**
   * calculate magnifier to perform undistortion
   *
   * @param radius_ratio radius ratio to max radius
   * @return radius magnifier
   * */
  template <typename Scalar>
  Scalar undistortion_magnifier(Scalar radius_ratio) const {
    Scalar magnitude = static_cast<Scalar>(data_(data_.rows() - 1));
    const int kLookUpTableStart = 4;
    int lookup_index = kLookUpTableStart;
    Precision radius_normalized = 0;

    while (lookup_index < data_.rows() and Scalar(radius_normalized) <= radius_ratio) {
      lookup_index++;
      radius_normalized = static_cast<Precision>(lookup_index - kLookUpTableStart) /
                          static_cast<Precision>(data_.rows() - (kLookUpTableStart + 1));
      radius_normalized *= 1.0_p + data_(lookup_index);
    }

    if (lookup_index < data_.rows()) {
      Precision prev_radius_normalized = static_cast<Precision>(lookup_index - 1 - kLookUpTableStart) /
                                         static_cast<Precision>(data_.rows() - (kLookUpTableStart + 1));
      prev_radius_normalized *= 1.0_p + data_(lookup_index - 1);

      Precision interpolation_coefficient =
          (radius_ratio - prev_radius_normalized) / (radius_normalized - prev_radius_normalized);
      magnitude = Scalar(data_(lookup_index) * interpolation_coefficient +
                         data_(lookup_index - 1) * (1.0 - interpolation_coefficient));
    }

    return Scalar(1) / (Scalar(1) + magnitude);
  }

  /**
   * calculate magnifier to perform distortion
   *
   * linear interpolation of radius magnifier from neighbour cells
   *
   * @param radius_ratio radius ratio to max radius
   * @return radius magnifier
   * */
  template <typename Scalar>
  Scalar distortion_magnifier(Scalar radius_ratio) const {
    if (radius_ratio < Scalar(1.0)) {
      const int kLookUpTableStart = 4;

      Scalar radius_index = radius_ratio * static_cast<Scalar>(data_.rows() - 5);
      long index = static_cast<long>(radius_index);

      Scalar interpolation_coefficient = radius_index - static_cast<Scalar>(index);

      Scalar magnifier1 = static_cast<Scalar>(data_(kLookUpTableStart + index));
      Scalar magnifier2 = static_cast<Scalar>(data_(kLookUpTableStart + index + 1));

      return (Scalar(1.0) - interpolation_coefficient) * magnifier1 + interpolation_coefficient * magnifier2 +
             Scalar(1.0);
    }
    return static_cast<Scalar>(data_(data_.rows() - 1)) + Scalar(1.0);
  }

  /** stores focal lengths, center and lookup talbe in specified order */
  Eigen::VectorX<Precision> data_;
  /** maximum radius of point inside image */
  Precision max_radius_;
};

}  // namespace dsopp::energy::model

#endif  // DSOPP_ENERGY_CAMERA_MODEL_PINHOLE_IOS_CAMERA_MODEL_HPP
