
#ifndef DSOPP_PINHOLE_CAMERA_HPP
#define DSOPP_PINHOLE_CAMERA_HPP

#include "common/settings.hpp"
#include "energy/camera_model/camera_model_base.hpp"
#include "energy/motion/motion.hpp"

namespace dsopp {
namespace energy {
namespace model {

/**
 * \brief Pinhole camera model
 *
 * Model to work with a pinhole camera model projective geometry.
 *
 * @tparam T type
 */
template <typename T = Precision>
class PinholeCamera : public CameraModelBase {
 public:
  /** Type of the model */
  static ModelType constexpr Type = ModelType::kPinholeCamera;
  /** alias to the base class with template */
  template <typename Scalar>
  using CastT = PinholeCamera<Scalar>;
  /** DoF in the intrinsics parameters*/
  static int constexpr DoF = 4;
  static_assert(Model<PinholeCamera<T>>);
  /**
   * creates Pinhole camera from the intrinsics parameters
   *
   * @param image_size width and height of the image
   * @param intrinsic_parameters focal lengths in pixels and coordinates of principal point in pixels
   * @param shutter_time shutter capture time
   * @param scale camera model scale
   */
  PinholeCamera(const Eigen::Vector2<Precision> &image_size, const Eigen::Vector<T, DoF> &intrinsic_parameters,
                time::duration shutter_time = CameraModelBase::kZeroShutterTime, const size_t scale = 1)
      : CameraModelBase(image_size, shutter_time, scale),
        focal_lengths_(intrinsic_parameters.template head<2>() / T(int(scale))),
        inverse_focal_lengths_(focal_lengths_.cwiseInverse()),
        principal_point_(intrinsic_parameters.template tail<2>() / T(int(scale))) {}
  /**
   * creates Pinhole camera from the focal lengths and coordinates of principal point
   *
   * @param image_size width and height of the image
   * @param focal_lengths focal lengths in pixels
   * @param principal_point coordinates of principal point in pixels
   * @param shutter_time shutter capture time
   * @param scale camera model scale
   */
  PinholeCamera(const Eigen::Vector2<Precision> &image_size, const Eigen::Vector2<T> &focal_lengths,
                const Eigen::Vector2<T> &principal_point,
                time::duration shutter_time = CameraModelBase::kZeroShutterTime, const size_t scale = 1)
      : CameraModelBase(image_size, shutter_time, scale),
        focal_lengths_(focal_lengths / T(scale)),
        inverse_focal_lengths_(focal_lengths_.cwiseInverse()),
        principal_point_(principal_point / T(scale)) {}

  /**
   * @return focal length in X axis
   */
  T focalX() const { return focal_lengths_(0); }
  /**
   * @return focal length in Y axis
   */
  T focalY() const { return focal_lengths_(1); }

  /**
   * projects 3d point into the image
   *
   * @param p_3d 3d point to be projected
   * @param[out] p_2d coordinate of the pixel on the image
   * @return true if the projection belongs to the image and false otherwise
   */
  template <typename EigenDerivedA, typename EigenDerivedB>
  bool project(const Eigen::MatrixBase<EigenDerivedA> &p_3d, Eigen::MatrixBase<EigenDerivedB> const &p_2d) const {
    static_assert(EigenDerivedA::RowsAtCompileTime == 3 and EigenDerivedA::ColsAtCompileTime == 1);
    static_assert(EigenDerivedB::RowsAtCompileTime == 2 and EigenDerivedB::ColsAtCompileTime == 1);
    static_assert(std::is_same_v<typename EigenDerivedA::Scalar, typename EigenDerivedB::Scalar>);
    using Scalar = typename EigenDerivedA::Scalar;
    const_cast<Eigen::MatrixBase<EigenDerivedB> &>(p_2d).noalias() =
        p_3d.template head<2>().cwiseProduct(focal_lengths_.template cast<Scalar>()) / p_3d[2] +
        principal_point_.template cast<Scalar>();
    if (p_3d[2] < Scalar(kMinDepth)) {
      return false;
    }
    return insideCameraROI(p_2d);
  }

  /**
   * projects 3d point into the image and evaluate jacobian
   *
   * @param p_3d 3d point to be projected
   * @param[out] p_2d coordinate of the pixel on the image
   * @param[out] jacobian jcaobian
   * @return true if the projection belongs to the image and false otherwise
   */
  template <class Scalar>
  bool project(const Eigen::Vector<Scalar, 3> &p_3d, Eigen::Vector<Scalar, 2> &p_2d,
               Eigen::Matrix<Scalar, 2, 3> &jacobian) const {
    p_2d = p_3d.template head<2>().cwiseProduct(focal_lengths_.template cast<Scalar>()) / p_3d[2] +
           principal_point_.template cast<Scalar>();

    Scalar inv_p_3d_sq = 1 / p_3d(2) / p_3d(2);
    jacobian(0, 0) = Scalar(focal_lengths_(0)) / p_3d(2);
    jacobian(0, 1) = Scalar(0);
    jacobian(0, 2) = -Scalar(focal_lengths_(0)) * p_3d(0) * inv_p_3d_sq;

    jacobian(1, 0) = Scalar(0);
    jacobian(1, 1) = Scalar(focal_lengths_(1)) / p_3d(2);
    jacobian(1, 2) = -Scalar(focal_lengths_(1)) * p_3d(1) * inv_p_3d_sq;

    if (p_3d[2] < Scalar(kMinDepth)) {
      return false;
    }
    return insideCameraROI(p_2d);
  }

  /**
   * calculates 3d line on which 3d point is lying for the given pixel
   *
   * @param p_2d coordinates of the pixel
   * @param[out] p_3d image plane vector (z = 1) pointing to the 3d point
   * @return true if the projection belongs to the image and false otherwise
   */
  template <typename EigenDerivedA, typename EigenDerivedB>
  bool unproject(const Eigen::MatrixBase<EigenDerivedA> &p_2d, Eigen::MatrixBase<EigenDerivedB> const &p_3d) const {
    static_assert(EigenDerivedA::RowsAtCompileTime == 2 and EigenDerivedA::ColsAtCompileTime == 1);
    static_assert(EigenDerivedB::RowsAtCompileTime == 3 and EigenDerivedB::ColsAtCompileTime == 1);
    static_assert(std::is_same_v<typename EigenDerivedA::Scalar, typename EigenDerivedB::Scalar>);
    using Scalar = typename EigenDerivedA::Scalar;
    if (not insideCameraROI(p_2d)) {
      return false;
    }
    const_cast<Eigen::MatrixBase<EigenDerivedB> &>(p_3d).template head<2>().noalias() =
        (p_2d - principal_point_.template cast<Scalar>()).cwiseProduct(inverse_focal_lengths_.template cast<Scalar>());
    const_cast<Eigen::MatrixBase<EigenDerivedB> &>(p_3d)[2] = Scalar(1);
    return true;
  }

  /** get focal lengths
   *
   * @return focal lengths
   */
  const Eigen::Vector2<T> &focal_lengths() const { return focal_lengths_; }

  /**
   * @return intrinsics parameters
   */
  Eigen::Vector<T, DoF> intrinsicsParameters() const {
    return {focal_lengths_(0), focal_lengths_(1), principal_point_(0), principal_point_(1)};
  }

  /** get principal point
   *
   * @return principal point
   */
  const Eigen::Vector2<T> &principal_point() const { return principal_point_; }

  /** transform intrinsic parameters to the string
   *
   * @return string with intrinsic parameters
   */
  std::string toString() const {
    return "pinhole\n" + std::to_string(this->image_size()(0)) + " " + std::to_string(this->image_size()(1)) + "\n" +
           std::to_string(focal_lengths_(0)) + " " + std::to_string(focal_lengths_(1)) + "\n" +
           std::to_string(principal_point_(0)) + " " + std::to_string(principal_point_(1));
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
    (void)fix_model_specific;
    std::vector<int> constant_parameters;
    if (fix_focal) constant_parameters = {0, 1};
    if (fix_center) {
      constant_parameters.push_back(2);
      constant_parameters.push_back(3);
    }
    return constant_parameters;
  }

 private:
  /** intrinsic parameters in order: focal lengths, principal point */
  Eigen::Vector<T, DoF> intrinsic_parameters_;
  /**  Focal lengths in pixels */
  const Eigen::Vector2<T> focal_lengths_;
  /**  Inverse focal length in pixels */
  const Eigen::Vector2<T> inverse_focal_lengths_;
  /** coordinates of principal point in pixels */
  const Eigen::Vector2<T> principal_point_;
};
}  // namespace model
}  // namespace energy
}  // namespace dsopp

#endif  // DSOPP_PINHOLE_CAMERA_HPP
