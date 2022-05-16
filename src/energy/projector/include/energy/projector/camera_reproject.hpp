#ifndef DSOPP_ENERGY_CAMERA_CAMERA_REPROJECT_HPP
#define DSOPP_ENERGY_CAMERA_CAMERA_REPROJECT_HPP

#include <Eigen/Dense>
#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"

#include "energy/projector/camera_projector.hpp"

namespace dsopp::energy::reprojection {

template <typename T, class CameraModel, energy::motion::MotionProduct MotionProduct, bool kCheckSuccess = true>
class ArrayReprojector;

template <class T>
struct traits;

/**
 * \brief specialization to extract  Scalar type for Base class
 * @tparam T scalar type
 * @tparam CameraModel camera model type
 * @tparam MotionProduct relative motion type
 * @tparam kCheckSuccess `true` if idepth, reprojection should be validated
 */
template <typename T, class CameraModel, energy::motion::MotionProduct MotionProduct, bool kCheckSuccess>
struct traits<ArrayReprojector<T, CameraModel, MotionProduct, kCheckSuccess>> {
  /** alias for scalar type */
  using Scalar = T;
  /** alias for Motion type */
  using MotionType = MotionProduct;
};

/**
 * \brief Base class for reprojection
 *
 * @tparam Derrived exact ArrayReprojector class
 */
template <class Derived>
class ArrayReprojectorBase {
 public:
  /** alias for Derived Scalar type */
  using Scalar = typename traits<Derived>::Scalar;
  /** alias for Derived Motion type */
  using Motion = typename traits<Derived>::MotionType;
  /**
   * reprojects Pattern to target frame
   *
   *
   * @param reference_points reference pattern points
   * @param idepth inverse depth
   * @param[out] target_points pattern points in target frame
   * @return true if reprojection was successfull
   */
  template <typename EigenDerivedA, typename EigenDerivedB>
  bool reprojectPattern(const Eigen::MatrixBase<EigenDerivedA> &reference_points, const Scalar &idepth,
                        Eigen::MatrixBase<EigenDerivedB> &target_points) const {
    static_assert(EigenDerivedA::RowsAtCompileTime == 2 and EigenDerivedB::RowsAtCompileTime == 2);
    return static_cast<Derived const *>(this)->reproject(reference_points, idepth, target_points);
  }

  /**
   * reprojects Pattern to target frame
   *
   * @tparam N size of a pattern
   *
   * @param reference_points reference pattern points
   * @param idepth inverse depth
   * @param[out] target_points pattern points in target frame\
   * @param[out] d_u_idepth,d_v_idepth jacobian of pattern coordinates with respect to inverse depth
   * @param[out] d_u_tReferenceTarget,d_v_tReferenceTarget jacobian of pattern coordinates with respect to target to
   * reference transformation
   * @return true if reprojection was successfull
   */
  template <typename EigenDerivedA, typename EigenDerivedB, typename EigenDerivedC, typename EigenDerivedD>
  bool reprojectPattern(const Eigen::MatrixBase<EigenDerivedA> &reference_points, const Scalar &idepth,
                        Eigen::MatrixBase<EigenDerivedB> &target_points, Eigen::MatrixBase<EigenDerivedC> &d_u_idepth,
                        Eigen::MatrixBase<EigenDerivedC> &d_v_idepth,
                        Eigen::MatrixBase<EigenDerivedD> &d_u_tReferenceTarget,
                        Eigen::MatrixBase<EigenDerivedD> &d_v_tReferenceTarget) const {
    static_assert(EigenDerivedA::RowsAtCompileTime == 2);
    static_assert(EigenDerivedB::RowsAtCompileTime == 2);
    static_assert(EigenDerivedC::ColsAtCompileTime == 1);
    static_assert(EigenDerivedD::ColsAtCompileTime == Motion::DoF);

    bool success = static_cast<Derived const *>(this)->reproject(
        reference_points, idepth, target_points, d_u_idepth, d_v_idepth, d_u_tReferenceTarget, d_v_tReferenceTarget);
    return success;
  }
};

/**
 * \brief class to perform point reprojections
 *
 * @tparam CameraModel camera model
 * @tparam T scalar type
 * @tparam MotionProduct relative motion type
 * @tparam kCheckSuccess `true` if idepth, reprojection should be validated
 */
template <typename T, class CameraModel, energy::motion::MotionProduct MotionProduct, bool kCheckSuccess>
class ArrayReprojector : public ArrayReprojectorBase<ArrayReprojector<T, CameraModel, MotionProduct, kCheckSuccess>> {
 public:
  /**
   * constructor
   * @param model camera model
   * @param t_target_reference motion from reference to target frame
   *
   */
  ArrayReprojector(const CameraModel &model, const MotionProduct &t_target_reference)
      : reference_model_(model), target_model_(model), t_target_reference_(t_target_reference) {}
  /**
   * constructor
   * @param reference_model reference camera model
   * @param target_model target camera model
   * @param t_target_reference motion from reference to target frame
   *
   */
  ArrayReprojector(const CameraModel &reference_model, const CameraModel &target_model,
                   const MotionProduct &t_target_reference)
      : reference_model_(reference_model), target_model_(target_model), t_target_reference_(t_target_reference) {}

  /**
   * Reprojects point from reference to target frame
   *
   * @param reference_point point in reference frame
   * @param idepth inverse depth
   * @param[out] _target_point point in target frame
   * @return true if reprojection was successfull
   */
  template <typename EigenDerivedA, typename EigenDerivedB>
  bool reproject(const Eigen::MatrixBase<EigenDerivedA> &reference_point, const T &idepth,
                 Eigen::MatrixBase<EigenDerivedB> const &_target_point) const {
    static_assert(EigenDerivedA::RowsAtCompileTime == 2 and EigenDerivedA::ColsAtCompileTime == 1);
    static_assert(EigenDerivedB::RowsAtCompileTime == 2 and EigenDerivedB::ColsAtCompileTime == 1);

    Eigen::Matrix<T, 3, 1> point_3d;
    bool success = true;
    if constexpr (kCheckSuccess) {
      success = reference_model_.insideCameraROI(reference_point);
      success = success && reference_model_.validIdepth(idepth);
    }

    success = success && reference_model_.unproject(reference_point, point_3d);

    point_3d = t_target_reference_.so3() * point_3d + idepth * t_target_reference_.translation();
    Eigen::Vector2<T> target_point;
    success = success && target_model_.template project<T>(point_3d, target_point);

    if constexpr (kCheckSuccess) {
      success &= target_model_.validIdepth(idepth / point_3d(2));
    }

    const_cast<Eigen::MatrixBase<EigenDerivedB> &>(_target_point) = target_point;

    return success;
  }
  /**
   * Reprojects point from reference to target frame
   *
   * @param reference_point point in reference frame
   * @param idepth inverse depth
   * @param[out] target_point point in target frame
   * @param[out] d_u_idepth, d_v_idepth jacobian of a target point with respect to idepth
   * @return true if reprojection was successfull
   */
  template <typename EigenDerivedA, typename EigenDerivedB, typename EigenDerivedC>
  bool reproject(const Eigen::MatrixBase<EigenDerivedA> &reference_point, const T &idepth,
                 Eigen::MatrixBase<EigenDerivedB> &target_point, double &d_u_idepth, double &d_v_idepth) const {
    (void)reference_point;
    (void)idepth;
    (void)target_point;
    (void)d_u_idepth;
    (void)d_v_idepth;
    throw std::runtime_error("not implemented");
    return false;
  }

 private:
  /** reference camera model */
  const CameraModel &reference_model_;
  /** target camera model */
  const CameraModel &target_model_;
  /** motion from reference to target frame */
  const MotionProduct &t_target_reference_;
};

/**
 * \brief class specialization to perform point reprojections
 * for pinhole camera with SE3 motion parameterization
 *
 * @tparam T scalar type
 * @tparam kCheckSuccess `true` if idepth, reprojection should be validated
 */
template <typename T, bool kCheckSuccess>
class ArrayReprojector<T, model::PinholeCamera<Precision>, energy::motion::SE3<T>, kCheckSuccess>
    : public ArrayReprojectorBase<
          ArrayReprojector<T, model::PinholeCamera<Precision>, energy::motion::SE3<T>, kCheckSuccess>> {
 public:
  /**
   * constructor, generates projectionMatrix
   *
   * @param model camera model
   * @param t_target_reference motion from reference to target frame
   */
  ArrayReprojector(const model::PinholeCamera<Precision> &model, const energy::motion::SE3<T> &t_target_reference)
      : reference_model_(model), target_model_(model) {
    auto focals = model.focal_lengths();
    auto center = model.principal_point();

    Eigen::Matrix<T, 3, 3> K = Eigen::Matrix<T, 3, 3>::Identity();
    K(0, 0) = T(focals(0));
    K(1, 1) = T(focals(1));
    K(0, 2) = T(center(0));
    K(1, 2) = T(center(1));

    Eigen::Matrix<T, 4, 4> Kinv = Eigen::Matrix<T, 4, 4>::Identity();
    Kinv(0, 0) = T(1 / focals(0));
    Kinv(1, 1) = T(1 / focals(1));
    Kinv(0, 2) = T(-center(0) / focals(0));
    Kinv(1, 2) = T(-center(1) / focals(1));

    reproject_ = K * t_target_reference.matrix3x4() * Kinv;
    project_ = K;
    transform_unproject_ = t_target_reference.matrix3x4() * Kinv;
    translation_ = t_target_reference.translation();
  }

  /**
   * constructor, generates projectionMatrix
   *
   * @param reference_model reference camera model
   * @param target_model target camera model
   * @param t_target_reference motion from reference to target frame
   */
  ArrayReprojector(const model::PinholeCamera<Precision> &reference_model,
                   const model::PinholeCamera<Precision> &target_model,
                   const energy::motion::SE3<T> &t_target_reference)
      : reference_model_(reference_model), target_model_(target_model) {
    auto reference_focals = reference_model.focal_lengths();
    auto reference_center = reference_model.principal_point();
    auto target_focals = target_model.focal_lengths();
    auto target_center = target_model.principal_point();

    Eigen::Matrix<T, 3, 3> K = Eigen::Matrix<T, 3, 3>::Identity();
    K(0, 0) = T(target_focals(0));
    K(1, 1) = T(target_focals(1));
    K(0, 2) = T(target_center(0));
    K(1, 2) = T(target_center(1));

    Eigen::Matrix<T, 4, 4> Kinv = Eigen::Matrix<T, 4, 4>::Identity();
    Kinv(0, 0) = T(1 / reference_focals(0));
    Kinv(1, 1) = T(1 / reference_focals(1));
    Kinv(0, 2) = T(-reference_center(0) / reference_focals(0));
    Kinv(1, 2) = T(-reference_center(1) / reference_focals(1));

    reproject_ = K * t_target_reference.matrix3x4() * Kinv;
    project_ = K;
    transform_unproject_ = t_target_reference.matrix3x4() * Kinv;
    translation_ = t_target_reference.translation();
  }

  /**
   * Reprojects point from reference to target frame
   *
   * @param reference_point point in reference frame
   * @param idepth inverse depth
   * @param[out] target_point point in target frame
   * @return true if reprojection was successfull
   */
  template <typename EigenDerivedA, typename EigenDerivedB>
  bool reproject(const Eigen::MatrixBase<EigenDerivedA> &reference_point, const T &idepth,
                 Eigen::MatrixBase<EigenDerivedB> &target_point) const {
    static_assert(EigenDerivedA::RowsAtCompileTime == 2);
    static_assert(EigenDerivedB::RowsAtCompileTime == 2);
    const int kPointsNumber = EigenDerivedA::ColsAtCompileTime;

    bool success = true;
    if constexpr (kCheckSuccess) {
      success = reference_model_.validIdepth(idepth) && reference_model_.insideCameraROI(reference_point);
    }
    constexpr auto Major = (kPointsNumber > 1 ? Eigen::RowMajor : Eigen::ColMajor);

    Eigen::Matrix<T, 3, kPointsNumber, Major> points_3d = reproject_.template topLeftCorner<3, 2>() * reference_point;
    points_3d.colwise() += reproject_.col(2) + reproject_.template topRightCorner<3, 1>() * idepth;

    target_point.noalias() = points_3d.colwise().hnormalized();
    if constexpr (kCheckSuccess) {
      success = success && (points_3d.row(2).array() > T(0)).all();
      success = success && target_model_.insideCameraROI(target_point);
    }

    return success;
  }
  /**
   * Reprojects point from reference to target frame
   *
   * @param reference_points point in reference frame
   * @param idepth inverse depth
   * @param[out] target_points point in target frame
   * @param[out] d_u_idepth,d_v_idepth jacobian of a target point with respect to idepth
   * @param[out] d_u_tReferenceTarget, d_v_tReferenceTarget jacobians of a target point with respect to target to
   * reference transformation
   * @return true if reprojection was successfull
   */
  template <typename EigenDerivedA, typename EigenDerivedB, typename EigenDerivedC, typename EigenDerivedD,
            typename EigenDerivedE, typename EigenDerivedF>
  bool reproject(const Eigen::MatrixBase<EigenDerivedA> &reference_points, const T &idepth,
                 Eigen::MatrixBase<EigenDerivedB> &target_points, Eigen::MatrixBase<EigenDerivedC> &d_u_idepth,
                 Eigen::MatrixBase<EigenDerivedD> &d_v_idepth, Eigen::MatrixBase<EigenDerivedE> &d_u_tReferenceTarget,
                 Eigen::MatrixBase<EigenDerivedF> &d_v_tReferenceTarget) const {
    static_assert(EigenDerivedA::RowsAtCompileTime == 2);
    static_assert(EigenDerivedB::RowsAtCompileTime == 2);
    static_assert(not EigenDerivedE::IsRowMajor or EigenDerivedE::RowsAtCompileTime == 1);
    static_assert(not EigenDerivedF::IsRowMajor or EigenDerivedF::RowsAtCompileTime == 1);
    const int kPointNum = EigenDerivedA::ColsAtCompileTime;

    bool success = true;
    if constexpr (kCheckSuccess) {
      success = reference_model_.validIdepth(idepth) && reference_model_.insideCameraROI(reference_points);
    }

    const auto Major = (kPointNum > 1 ? Eigen::RowMajor : Eigen::ColMajor);
    Eigen::Matrix<T, 3, kPointNum, Major> points_3d =
        transform_unproject_.template topLeftCorner<3, 2>() * reference_points;
    points_3d.colwise() += transform_unproject_.col(2) + transform_unproject_.template topRightCorner<3, 1>() * idepth;

    if constexpr (kCheckSuccess) {
      success = success && (points_3d.row(2).array() > T(0)).all();
    }

    const Eigen::Matrix<T, 3, kPointNum, Major> projected = project_ * points_3d;
    target_points.row(0).noalias() = (projected.row(0).array() / projected.row(2).array()).matrix();
    target_points.row(1).noalias() = (projected.row(1).array() / projected.row(2).array()).matrix();

    if constexpr (kCheckSuccess) {
      success = success && target_model_.insideCameraROI(target_points);
    }

    const Eigen::Matrix<T, 1, kPointNum> rescalings = points_3d.template bottomRows<1>().cwiseInverse();
    const Eigen::Matrix<T, 1, kPointNum> b0 = (points_3d.row(0).array() * rescalings.array()).matrix();
    const Eigen::Matrix<T, 1, kPointNum> b1 = (points_3d.row(1).array() * rescalings.array()).matrix();

    d_u_idepth.noalias() =
        project_(0, 0) * (translation_[0] * rescalings - translation_[2] * rescalings.cwiseProduct(b0));
    d_v_idepth.noalias() =
        project_(1, 1) * (translation_[1] * rescalings - translation_[2] * rescalings.cwiseProduct(b1));

    const Eigen::Matrix<T, 1, kPointNum> new_idepths = idepth * rescalings;

    d_v_tReferenceTarget.col(0).setConstant(0);
    d_v_tReferenceTarget.col(1).noalias() = new_idepths;
    d_v_tReferenceTarget.col(2).noalias() = -new_idepths.cwiseProduct(b1);
    d_v_tReferenceTarget.col(3).noalias() = -(b1.array().square() + 1).matrix();
    d_v_tReferenceTarget.col(4).noalias() = b0.cwiseProduct(b1);
    d_v_tReferenceTarget.col(5).noalias() = b0;

    d_u_tReferenceTarget.col(0).noalias() = new_idepths;
    d_u_tReferenceTarget.col(1).setConstant(0);
    d_u_tReferenceTarget.col(2).noalias() = -new_idepths.cwiseProduct(b0);
    d_u_tReferenceTarget.col(3).noalias() = -d_v_tReferenceTarget.col(4);
    d_u_tReferenceTarget.col(4).noalias() = (b0.array().square() + 1).matrix();
    d_u_tReferenceTarget.col(5).noalias() = -b1;

    d_u_tReferenceTarget *= project_(0, 0);
    d_v_tReferenceTarget *= project_(1, 1);
    return success;
  }

 private:
  /** camera reprojection matrix acts on [u, v, 1, idepth], eqviualent to project_ * transform_unproject_*/
  Eigen::Matrix<T, 3, 4> reproject_;
  /** camera projection matrix acts on [x, y, z] */
  Eigen::Matrix<T, 3, 3> project_;
  /** matrix for unprojectionion and transformation of a point, acts on [u, v, 1, idepth] */
  Eigen::Matrix<T, 3, 4> transform_unproject_;
  /** translation component of t_target_reference */
  Eigen::Vector<T, 3> translation_;
  /** reference camera model */
  const model::PinholeCamera<Precision> &reference_model_;
  /** target camera model */
  const model::PinholeCamera<Precision> &target_model_;
};

/**
 * \brief class specialization to perform point reprojections
 * for general camera with SE3 motion
 *
 * @tparam T scalar type
 * @tparam CameraModel cmaera model type
 * @tparam kCheckSuccess `true` if idepth, reprojection should be validated
 */
template <typename T, class CameraModel, bool kCheckSuccess>
class ArrayReprojector<T, CameraModel, energy::motion::SE3<T>, kCheckSuccess>
    : public ArrayReprojectorBase<ArrayReprojector<T, CameraModel, energy::motion::SE3<T>, kCheckSuccess>> {
 public:
  /**
   * constructor
   *
   * @param model camera model
   * @param t_target_reference motion from reference to target frame
   */
  ArrayReprojector(const CameraModel &model, const energy::motion::SE3<T> &t_target_reference)
      : reference_model_(model), target_model_(model), t_target_reference_(t_target_reference.matrix3x4()) {}

  /**
   * constructor
   *
   * @param reference_model reference camera model
   * @param target_model target camera model
   * @param t_target_reference motion from reference to target frame
   */
  ArrayReprojector(const CameraModel &reference_model, const CameraModel &target_model,
                   const energy::motion::SE3<T> &t_target_reference)
      : reference_model_(reference_model),
        target_model_(target_model),
        t_target_reference_(t_target_reference.matrix3x4()) {}

  /**
   * Reprojects point from reference to target frame
   *
   * @param reference_point point in reference frame
   * @param idepth inverse depth
   * @param[out] _target_points point in target frame
   * @return true if reprojection was successfull
   */
  template <typename EigenDerivedA, typename EigenDerivedB>
  bool reproject(const Eigen::MatrixBase<EigenDerivedA> &reference_point, const T &idepth,
                 Eigen::MatrixBase<EigenDerivedB> const &_target_points) const {
    static_assert(EigenDerivedA::RowsAtCompileTime == 2);
    static_assert(EigenDerivedB::RowsAtCompileTime == 2);
    const int kPointsNumber = EigenDerivedA::ColsAtCompileTime;

    if constexpr (kPointsNumber == 1) {
      return reprojectPoint(reference_point, idepth, _target_points);
    } else {
      bool success = true;

      Eigen::MatrixBase<EigenDerivedB> &target_points = const_cast<Eigen::MatrixBase<EigenDerivedB> &>(_target_points);
      for (int point_i = 0; point_i < kPointsNumber && success; ++point_i) {
        success &= reprojectPoint(reference_point.col(point_i), idepth, target_points.col(point_i));
      }
      return success;
    }
  }
  /**
   * Reprojects one point from reference to target frame

   * @param reference_point point in reference frame
   * @param idepth inverse depth
   * @param[out] _target_point point in target frame
   * @param[out] d_u_idepth,d_v_idepth jacobian of a target point with respect to idepth
   * @param[out] d_u_tReferenceTarget, d_v_tReferenceTarget jacobians of a target point with respect to target to
   * reference transformation
   * @return true if reprojection was successfull
   */
  template <typename EigenDerivedA, typename EigenDerivedB, typename EigenDerivedC, typename EigenDerivedD>
  bool reprojectPoint(const Eigen::MatrixBase<EigenDerivedA> &reference_point, const T &idepth,
                      const Eigen::MatrixBase<EigenDerivedB> &_target_point, T &d_u_idepth, T &d_v_idepth,
                      const Eigen::MatrixBase<EigenDerivedC> &d_u_tReferenceTarget,
                      const Eigen::MatrixBase<EigenDerivedD> &d_v_tReferenceTarget) const {
    static_assert(EigenDerivedA::RowsAtCompileTime == 2);
    static_assert(EigenDerivedB::RowsAtCompileTime == 2);
    static_assert(EigenDerivedA::ColsAtCompileTime == 1);
    static_assert(EigenDerivedB::ColsAtCompileTime == 1);

    static_assert(EigenDerivedA::RowsAtCompileTime == 2);
    static_assert(EigenDerivedB::RowsAtCompileTime == 2);
    static_assert(EigenDerivedA::ColsAtCompileTime == 1);
    static_assert(EigenDerivedB::ColsAtCompileTime == 1);

    bool success = true;

    if constexpr (kCheckSuccess) {
      success = reference_model_.insideCameraROI(reference_point);
      success = success && reference_model_.validIdepth(idepth);
    }

    Eigen::Matrix<T, 3, 1> point_3d;

    success = success && reference_model_.unproject(reference_point, point_3d);

    point_3d = t_target_reference_.template leftCols<3>() * point_3d + idepth * t_target_reference_.col(3);

    Eigen::Matrix<T, 3, energy::motion::SE3<T>::DoF> d_transform;
    d_transform.template block<3, 3>(0, 0) = idepth * Eigen::Matrix<T, 3, 3>::Identity();
    d_transform.template block<3, 3>(0, 3) = -Sophus::SO3<T>::hat(point_3d);

    if constexpr (kCheckSuccess) {
      success = success && target_model_.validIdepth(idepth / point_3d(2));
    }

    Eigen::Vector<T, 2> target_point;
    Eigen::Matrix<T, 2, 3> jacobian;
    success = success && target_model_.project(point_3d, target_point, jacobian);

    Eigen::Vector<T, 2> d_uv_idepth = jacobian * t_target_reference_.col(3);
    d_u_idepth = d_uv_idepth(0);
    d_v_idepth = d_uv_idepth(1);

    Eigen::Matrix<T, 2, energy::motion::SE3<T>::DoF> d_uv_tReferenceTarget = jacobian * d_transform;

    const_cast<Eigen::MatrixBase<EigenDerivedC> &>(d_u_tReferenceTarget) = d_uv_tReferenceTarget.row(0);
    const_cast<Eigen::MatrixBase<EigenDerivedD> &>(d_v_tReferenceTarget) = d_uv_tReferenceTarget.row(1);

    const_cast<Eigen::MatrixBase<EigenDerivedB> &>(_target_point) = target_point;

    return success;
  }
  /**
   * Reprojects point from reference to target frame

   * @param reference_points point in reference frame
   * @param idepth inverse depth
   * @param[out] _target_points point in target frame
   * @param[out] d_u_idepth,d_v_idepth jacobian of a target point with respect to idepth
   * @param[out] _d_u_tReferenceTarget, _d_v_tReferenceTarget jacobians of a target point with respect to target to
   * reference transformation
   * @return true if reprojection was successfull
   */
  template <typename EigenDerivedA, typename EigenDerivedB, typename EigenDerivedC, typename EigenDerivedD,
            typename EigenDerivedE, typename EigenDerivedF>
  bool reproject(const Eigen::MatrixBase<EigenDerivedA> &reference_points, const T &idepth,
                 const Eigen::MatrixBase<EigenDerivedB> &_target_points, Eigen::MatrixBase<EigenDerivedC> &d_u_idepth,
                 Eigen::MatrixBase<EigenDerivedD> &d_v_idepth,
                 const Eigen::MatrixBase<EigenDerivedE> &_d_u_tReferenceTarget,
                 const Eigen::MatrixBase<EigenDerivedF> &_d_v_tReferenceTarget) const {
    static_assert(EigenDerivedA::RowsAtCompileTime == 2);
    static_assert(EigenDerivedB::RowsAtCompileTime == 2);
    const int kPointNum = EigenDerivedA::ColsAtCompileTime;

    if constexpr (kPointNum == 1) {
      return reprojectPoint(reference_points, idepth, _target_points, d_u_idepth(0), d_v_idepth(0),
                            _d_u_tReferenceTarget, _d_v_tReferenceTarget);
    } else {
      bool success = true;

      Eigen::MatrixBase<EigenDerivedB> &target_points = const_cast<Eigen::MatrixBase<EigenDerivedB> &>(_target_points);

      Eigen::MatrixBase<EigenDerivedE> &d_u_tReferenceTarget =
          const_cast<Eigen::MatrixBase<EigenDerivedE> &>(_d_u_tReferenceTarget);
      Eigen::MatrixBase<EigenDerivedF> &d_v_tReferenceTarget =
          const_cast<Eigen::MatrixBase<EigenDerivedF> &>(_d_v_tReferenceTarget);

      for (int point_i = 0; point_i < kPointNum && success; ++point_i) {
        success &=
            reprojectPoint(reference_points.col(point_i), idepth, target_points.col(point_i), d_u_idepth(point_i),
                           d_v_idepth(point_i), d_u_tReferenceTarget.row(point_i), d_v_tReferenceTarget.row(point_i));
      }
      return success;
    }
  }

  /**
   * Reprojects one point from reference to target frame
   *
   * @param reference_point point in reference frame
   * @param idepth inverse depth
   * @param[out] _target_point point in target frame
   * @return true if reprojection was successfull
   */
  template <typename EigenDerivedA, typename EigenDerivedB>
  bool reprojectPoint(const Eigen::MatrixBase<EigenDerivedA> &reference_point, const T &idepth,
                      Eigen::MatrixBase<EigenDerivedB> const &_target_point) const {
    static_assert(EigenDerivedA::RowsAtCompileTime == 2);
    static_assert(EigenDerivedB::RowsAtCompileTime == 2);
    static_assert(EigenDerivedA::ColsAtCompileTime == 1);
    static_assert(EigenDerivedB::ColsAtCompileTime == 1);

    bool success = true;
    if constexpr (kCheckSuccess) {
      success = reference_model_.insideCameraROI(reference_point);
      success = success && reference_model_.validIdepth(idepth);
    }

    Eigen::Matrix<T, 3, 1> point_3d;

    success = success && reference_model_.unproject(reference_point, point_3d);

    point_3d = t_target_reference_.template leftCols<3>() * point_3d + idepth * t_target_reference_.col(3);

    if constexpr (kCheckSuccess) {
      success = success && target_model_.validIdepth(idepth / point_3d(2));
    }

    Eigen::Vector<T, 2> target_point;
    success = success && target_model_.project(point_3d, target_point);

    const_cast<Eigen::MatrixBase<EigenDerivedB> &>(_target_point) = target_point;

    return success;
  }

 private:
  /** reference camera model */
  const CameraModel &reference_model_;
  /** target camera model */
  const CameraModel &target_model_;
  /** tranformation matrix */
  Eigen::Matrix<T, 3, 4> t_target_reference_;
};

}  // namespace dsopp::energy::reprojection

#endif  // DSOPP_ENERGY_CAMERA_CAMERA_REPROJECT_HPP
