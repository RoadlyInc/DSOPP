#ifndef DSOPP_ENERGY_CAMERA_MODEL_CAMERA_MODEL_HPP_
#define DSOPP_ENERGY_CAMERA_MODEL_CAMERA_MODEL_HPP_

#include <Eigen/Dense>

#include "common/settings.hpp"
#include "common/time/time.hpp"
#include "energy/motion/motion.hpp"

namespace dsopp::energy::model {

enum ModelType { kPinholeCamera, kSimpleRadialCamera, kDivisionModelCamera, kTumFovCamera };
/**
 * \brief Model concept.
 *
 * Camera should know it's type and number of the intrinsics parameters.
 */
template <typename ConstraintModel>
concept Model = requires(const ConstraintModel &model) {
  { ConstraintModel::Type }
  ->std::convertible_to<ModelType>;
  { ConstraintModel::DoF }
  ->std::convertible_to<int>;
};

/**
 * \brief Base class for camera models
 *
 */
class CameraModelBase {
 public:
  // TODO: move it to PatternPatch
  /** Border size for project/unproject */
  static constexpr Precision kBorderSize = 4_p;
  /** minimal depth for projection*/
  static constexpr Precision kMinDepth = 0.001_p;
  /** default shutter time (0) captures immediately */
  static constexpr time::duration kZeroShutterTime = time::duration(0);
  /**
   * @param image_size size of image
   * @param shutter_time shutter_time
   * @param scale camera model scale
   */
  CameraModelBase(const Eigen::Vector2<Precision> &image_size, time::duration shutter_time, const size_t scale = 1);

  /**
   * checks if point is inside camera ROI
   *
   * @param points 2d points in image plane (matrix 2 x N)
   * @return true if inside ROI
   */
  template <typename EigenDerived>
  bool insideCameraROI(const Eigen::MatrixBase<EigenDerived> &points) const {
    static_assert(EigenDerived::RowsAtCompileTime == 2);
    using Scalar = typename EigenDerived::Scalar;
    return (points.template topRows<1>().array() >= Scalar(kBorderSize)).all() and
           (points.template bottomRows<1>().array() >= Scalar(kBorderSize)).all() and
           (points.template topRows<1>().array() <= (Scalar(image_size_[0] - kBorderSize - 1))).all() and
           (points.template bottomRows<1>().array() <= (Scalar(image_size_[1] - kBorderSize - 1))).all();
  }
  /**
   * checks if inverse depth is within threshold
   *
   * @param idepth inverse depth
   * @return check flag
   */
  template <typename Scalar>
  bool validIdepth(const Scalar &idepth) const {
    constexpr Precision kMinIdepthEps = 1e-4_p;
    constexpr Precision kMaxIdepthEps = 1e1;
    constexpr Precision kMinIdepth = -kMinIdepthEps;
    constexpr Precision kMaxIdepth = 1 / kMinDepth + kMaxIdepthEps;
    return idepth > Scalar(kMinIdepth) && idepth < Scalar(kMaxIdepth);
  }

  /**
   * checks if depth is within threshold
   *
   * @param depth depth
   * @return check flag
   */
  template <typename Scalar>
  bool validDepth(const Scalar &depth) const {
    return (depth >= kMinDepth);
  }

  /** get image size
   *
   * @return image size
   */
  const Eigen::Vector2<Precision> &image_size() const;

  /**
   * calculate the relative change in depth during transformation. It's the ratio between the idepth of the landmark in
   * the reference and target frames.
   *
   * @param point_3d bearing vector of the given pixel
   * @param idepth inverse depth of the 3d point that projected on the given pixel
   * @param t_r_t homogeneous transformation between camera positions
   * @return relative change in depth during transformation
   */
  template <energy::motion::MotionProduct MotionProduct>
  Precision getDepthScale(const Eigen::Vector3<Precision> &point_3d, Precision idepth, const MotionProduct &t_r_t) {
    Eigen::Vector4<Precision> point_homogeneous;
    point_homogeneous << point_3d, idepth;
    return (t_r_t * point_homogeneous)(2);
  }

  /**
   * @return shutter_time
   */
  time::duration shutterTime() const;

 protected:
  /** Width and height of the image*/
  const Eigen::Vector2<Precision> image_size_;
  /** shutter time */
  const time::duration shutter_time_;
};

}  // namespace dsopp::energy::model

#endif  // DSOPP_ENERGY_CAMERA_MODEL_CAMERA_MODEL_HPP_
