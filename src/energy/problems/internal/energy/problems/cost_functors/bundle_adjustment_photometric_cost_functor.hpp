#ifndef DSOPP_SRC_ENERGY_PROBLEMS_COST_FUNCTORS_BUNDLE_ADJUSTMENT_PHOTOMETRIC_COST_FUNCTOR_HPP_
#define DSOPP_SRC_ENERGY_PROBLEMS_COST_FUNCTORS_BUNDLE_ADJUSTMENT_PHOTOMETRIC_COST_FUNCTOR_HPP_

#include <ceres/ceres.h>
#include <Eigen/Dense>

#include "common/patch/patch.hpp"
#include "common/pattern/pattern.hpp"
#include "energy/motion/motion.hpp"
#include "energy/projector/camera_reproject.hpp"
#include "features/camera/pattern_patch.hpp"
#include "features/camera/pixel_map.hpp"
#include "measures/similarity_measure_ssd.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"
#include "sensors/camera_calibration/mask/camera_mask.hpp"
#include "track/connections/frame_connection.hpp"

namespace dsopp {
namespace energy {
namespace problem {
namespace cost_functors {
/** \brief Functor to calculate the cost of photometric bundle adjustment between two frames
 * for given landmark
 *
 * @tparam Grid2D 2D grid to evaluate data
 */
template <energy::motion::Motion Motion, typename Grid2D, typename Model, int PatternSize, int C>
class BundleAdjustmentPhotometricCostFunctor {
 public:
  /** Caster motion for ceres solver*/
  using MotionDouble = typename Motion::template CastT<double>;

  /** number of residuals */
  static constexpr int residuals_num = PatternSize * C;
  /**
   * Create functor to calculate the cost of photometric bundle adjustment between two frames
   * @param t_w_r0,t_w_t0 initial estimates of camera tranformation
   * @param reference_affine_brightness0,target_affine_brightness0 initial estimates of affine brigtensses
   * @param target_grid 2D grid corresponding to the target frame
   * @param reference_pattern projection of the landmark on the reference frame with pattern
   * @param patch patch of the landmarks on the reference frame
   * @param mask masking for excluding region
   * @param reference_image_size,target_image_size image size
   * */
  BundleAdjustmentPhotometricCostFunctor(const MotionDouble &t_w_r0, const MotionDouble &t_w_t0,
                                         const Eigen::Vector2d &reference_affine_brightness0,
                                         const Eigen::Vector2d &target_affine_brightness0, const Grid2D &target_grid,
                                         const Eigen::Matrix<double, 2, PatternSize> &reference_pattern,
                                         const Eigen::Matrix<double, PatternSize, C, PatchStorageOrder<C>> &patch,
                                         const sensors::calibration::CameraMask &mask,
                                         const Eigen::Vector2<Precision> &reference_image_size,
                                         const Eigen::Vector2<Precision> &target_image_size)
      : t_t_r0(t_w_t0.inverse() * t_w_r0),
        reference_affine_brightness0_(reference_affine_brightness0),
        target_affine_brightness0_(target_affine_brightness0),
        reference_pattern_(reference_pattern),
        patch_(patch),
        target_grid_(target_grid),
        mask_(mask),
        reference_image_size_(reference_image_size),
        target_image_size_(target_image_size) {}
  template <class SE3T, class AffBrightness, class T, class ModelT, class Result>
  /**
   * Operator overloading for the ceres optimization with poses and idepths
   * @param t_w_r_eps additive value to a reference pose
   * @param t_w_t_eps additive value to atarget pose
   * @param reference_affine_brightness_eps additive value to a reference pose's affine brightess
   * @param target_affine_brightness_eps additive value to a reference pose's affine brightess
   * @param idepth inverse depth of the landmark
   * @param reference_intrinsic_parameters,target_intrinsic_parameters intrinsic parameters
   * @param residuals cost to fill out
   * @return reprojection status
   */
  track::PointConnectionStatus operator()(const SE3T &t_w_r_eps, const SE3T &t_w_t_eps, const T &idepth,
                                          const AffBrightness &reference_affine_brightness_eps,
                                          const AffBrightness &target_affine_brightness_eps,
                                          const ModelT &reference_intrinsic_parameters,
                                          const ModelT &target_intrinsic_parameters, Result &residuals) const {
    using MotionJet = decltype(typename Motion::Product().template cast<T>());
    using ModelJet = typename Model::template CastT<T>;

    MotionJet t_target_reference = t_t_r0.template cast<T>().rightIncrement(t_w_r_eps).leftIncrement(-t_w_t_eps);
    auto reference_affine_brightness = reference_affine_brightness0_.cast<T>() + reference_affine_brightness_eps;
    auto target_affine_brightness = target_affine_brightness0_.cast<T>() + target_affine_brightness_eps;

    Eigen::Matrix<T, 2, PatternSize> target_pattern;

    ModelJet reference_model(reference_image_size_, reference_intrinsic_parameters);
    ModelJet target_model(target_image_size_, target_intrinsic_parameters);

    reprojection::ArrayReprojector<T, ModelJet, MotionJet> reprojector(reference_model, target_model,
                                                                       t_target_reference);

    bool success = reprojector.reprojectPattern(reference_pattern_.template cast<T>(), idepth, target_pattern);

    const bool kCheckBoundaries = false;
    success = success && mask_.template valid<kCheckBoundaries>(target_pattern);

    if (success) {
      Eigen::Matrix<T, PatternSize, C, PatchStorageOrder<C>> target_patch;
      target_grid_.Evaluate(target_pattern, target_patch);
      Eigen::Map<Eigen::Matrix<T, PatternSize, C, PatchStorageOrder<C>>> matrix_residuals(residuals.data());
      measure::SimilarityMeasureSSD::residuals(
          target_patch - Eigen::Matrix<T, PatternSize, C, PatchStorageOrder<C>>::Constant(target_affine_brightness[1]),
          ceres::exp(target_affine_brightness[0] - reference_affine_brightness[0]) *
              (patch_.template cast<T>() -
               Eigen::Matrix<T, PatternSize, C, PatchStorageOrder<C>>::Constant(reference_affine_brightness[1])),
          matrix_residuals);
      return track::PointConnectionStatus::kOk;
    }
    residuals.setZero();
    return track::PointConnectionStatus::kOOB;
  }

  /**
   * Operator overloading for the ceres optimization with poses and idepths
   * @param reference_state_eps_ptr log of reference pose
   * @param target_state_eps_ptr log of target pose
   * @param idepth inverse depth of the landmark
   * @param reference_intrinsic_parameters_ptr,target_intrinsic_parameters_ptr intrinsic parameters
   * @param residuals_ptr cost to fill out
   * @return true
   */
  template <class T>
  bool operator()(T const *const reference_state_eps_ptr, T const *const target_state_eps_ptr, const T *const idepth,
                  T const *const reference_intrinsic_parameters_ptr, T const *const target_intrinsic_parameters_ptr,
                  T *residuals_ptr) const {
    Eigen::Map<Eigen::Matrix<T, Motion::DoF, 1> const> const t_log_w_r(reference_state_eps_ptr);
    Eigen::Map<Eigen::Matrix<T, Motion::DoF, 1> const> const t_log_w_t(target_state_eps_ptr);
    Eigen::Map<Eigen::Matrix<T, 2, 1> const> const reference_affine_brightness(reference_state_eps_ptr + Motion::DoF);
    Eigen::Map<Eigen::Matrix<T, 2, 1> const> const target_affine_brightness(target_state_eps_ptr + Motion::DoF);
    Eigen::Map<Eigen::Matrix<T, Model::DoF, 1> const> const reference_intrinsic_parameters(
        reference_intrinsic_parameters_ptr);
    Eigen::Map<Eigen::Matrix<T, Model::DoF, 1> const> const target_intrinsic_parameters(
        target_intrinsic_parameters_ptr);
    Eigen::Map<Eigen::Matrix<T, residuals_num, 1>> residuals(residuals_ptr);

    (*this)(t_log_w_r, t_log_w_t, *idepth, reference_affine_brightness, target_affine_brightness,
            reference_intrinsic_parameters, target_intrinsic_parameters, residuals);
    return true;
  }

 private:
  /** initial estimate of reference to target transformation*/
  const typename MotionDouble::Product t_t_r0;
  /** initial estimate of referenece frame affine brightess */
  const Eigen::Vector2d reference_affine_brightness0_;
  /** initial estimate of target frame affine brightess */
  const Eigen::Vector2d target_affine_brightness0_;
  /** patterrn of the landmark */
  const Eigen::Matrix<double, 2, PatternSize> reference_pattern_;
  /** patch of the landmark */
  const Eigen::Matrix<double, PatternSize, C, PatchStorageOrder<C>> patch_;
  /** 2D grids corresponding to the target frames */
  const Grid2D &target_grid_;
  /** camera mask */
  const sensors::calibration::CameraMask &mask_;
  /** image size */
  const Eigen::Vector2<Precision> &reference_image_size_;
  /** image size */
  const Eigen::Vector2<Precision> &target_image_size_;
};
}  // namespace cost_functors
}  // namespace problem
}  // namespace energy
}  // namespace dsopp

#endif  // DSOPP_SRC_ENERGY_PROBLEMS_COST_FUNCTORS_BUNDLE_ADJUSTMENT_PHOTOMETRIC_COST_FUNCTOR_HPP_
