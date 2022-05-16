
#ifndef DSOPP_ENERGY_CAMERA_MODEL_FIHEYE_ATAN_CAMERA_HPP
#define DSOPP_ENERGY_CAMERA_MODEL_FIHEYE_ATAN_CAMERA_HPP

#include "common/settings.hpp"
#include "energy/camera_model/camera_model_base.hpp"

#include <Eigen/Dense>

namespace dsopp::energy::model {

namespace {

/**
 * Function to evaluate first minimum root of polynomial
 *
 * @param polynomial polynomial
 * @return first minimum root
 */
template <typename T>
T minimalPositiveRoot(const Eigen::Matrix<T, -1, 1> &polynomial) {
  long N = polynomial.rows();

  if (polynomial.rows() == 2) return polynomial(0);

  Eigen::Matrix<T, -1, -1> companion(N - 1, N - 1);
  companion.setZero();
  companion.block(0, N - 2, N - 1, 1) = -polynomial.head(N - 1) / polynomial[N - 1];
  companion.block(1, 0, N - 2, N - 2).diagonal().setOnes();

  auto lambda = (companion.eigenvalues().eval());

  T ans = T(std::numeric_limits<Precision>::infinity());

  for (int i = 0; i < lambda.rows(); ++i) {
    auto l = lambda[i];

    if (abs(l.imag()) > Eigen::NumTraits<T>::epsilon() * abs(l.real()) || l.real() < T(0.0)) continue;
    ans = std::min(ans, l.real());
  }

  return ans;
}

}  // namespace

/**
 * \brief Atan camera model
 *
 * Model to work with fisheye camera, can project points with `z=0`
 * works slow for projecting pixel to ray (need to find root of polynomial)
 * Possible improvement : estimateion of back projection polynomial
 *
 * model similar to https://docs.opencv.org/master/db/d58/group__calib3d__fisheye.html
 * but with `atan2` modification to support `z < 0`
 *
 * @tparam Scalar type
 */
class AtanCamera : public CameraModelBase {
 public:
  /** alias for templated Eigen vector */
  template <typename T, int N>
  using Vec = Eigen::Matrix<T, N, 1>;

  /** Default destructor */
  ~AtanCamera() = default;

  /**
   * @return focal length in X axis
   */
  Precision focalX() const;
  /**
   * @return focal length in Y axis
   */
  Precision focalY() const;

  /**
   * creates Atan camera model
   *
   * @param image_size image size
   * @param focal_lengths focal lengths
   * @param center projection center
   * @param polynomial coeffs of radius(angle) distortion polynomial
   * @param shutter_time shutter capture time
   */
  AtanCamera(const Eigen::Vector2<Precision> &image_size, const Eigen::Vector2<Precision> &focal_lengths,
             const Eigen::Vector2<Precision> &center, const Eigen::VectorX<Precision> &polynomial,
             time::duration shutter_time = CameraModelBase::kZeroShutterTime);

  /**
   * projection templated method
   *
   * @param ray_  ray to project to 2d point
   * @param point output value
   * @return bool if proejction was successful
   */
  template <typename Scalar>
  bool project(const Vec<Scalar, 3> ray_, Vec<Scalar, 2> &point) const {
    Vec<Scalar, 3> ray = ray_.normalized();

    Scalar radius = ray.template head<2>().norm();
    if (radius < Scalar(1e-6)) {
      point = data_.template segment<2>(2).cast<Scalar>();
      return true;
    }

    Scalar theta = std::atan2(radius, ray(2));
    Scalar theta_step = theta;

    Scalar radius_new = Scalar(0.0);

    const int kPolynomialStart = 4;
    for (long i = data_.rows() - 1; i >= kPolynomialStart; --i) {
      radius_new = radius_new * theta_step + Scalar(data_(i));
    }

    radius_new = theta_step * (Scalar(1.0) + radius_new * theta_step);

    auto focals = data_.template head<2>().cast<Scalar>().array();
    auto center = data_.template segment<2>(2).cast<Scalar>();

    point = ray.template head<2>() * radius_new / radius;
    point = point.array() * focals;
    point = point + center;

    return insideCameraROI(point);
  }

  /**
   * unprojection templated method
   *
   * @param point_ 2d point to project to ray
   * @param ray output value
   * @return bool if proejction was successful
   */
  template <typename Scalar>
  bool unproject(const Vec<Scalar, 2> point_, Vec<Scalar, 3> &ray) const {
    auto focals = data_.template head<2>().cast<Scalar>().array();
    auto center = data_.template segment<2>(2).cast<Scalar>();

    Vec<Scalar, 2> point = point_ - center;
    point = point.array() / focals;

    Scalar radius = point.norm();
    if (radius < Scalar(1e-6)) {
      ray = point.homogeneous();
      return true;
    }
    const int kPolynomialStart = 4;

    Vec<Scalar, -1> polynomial(data_.rows() - kPolynomialStart + 2);
    polynomial.setZero();

    polynomial(0) = -radius;
    polynomial(1) = Scalar(1.0);

    for (int i = kPolynomialStart; i < data_.rows(); ++i) {
      polynomial(i - 2) = Scalar(data_(i));
    }

    Scalar root = minimalPositiveRoot(polynomial);
    ray.template head<2>() = point * sin(root) / radius;
    ray(2) = std::cos(root);
    return true;
  }

 private:
  /** stores focal lengths, center and polynomial in specified order */
  Eigen::VectorX<Precision> data_;
};

}  // namespace dsopp::energy::model

#endif  // DSOPP_ENERGY_CAMERA_MODEL_FIHEYE_ATAN_CAMERA_HPP
