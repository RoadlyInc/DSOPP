#ifndef DSOPP_BUNDLE_ADJUSTMENT_PHOTOMETRIC_COST_FUNCTOR_ANALYTIC_HPP
#define DSOPP_BUNDLE_ADJUSTMENT_PHOTOMETRIC_COST_FUNCTOR_ANALYTIC_HPP

#include "energy/motion/motion.hpp"
#include "energy/problems/photometric_bundle_adjustment/local_frame.hpp"
#include "energy/projector/camera_reproject.hpp"
#include "features/camera/pattern_patch.hpp"
#include "features/camera/pixel_map.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"
#include "sensors/camera_calibration/mask/camera_mask.hpp"

#include <ceres/ceres.h>
#include <Eigen/Dense>

namespace dsopp {
namespace energy {
namespace problem {
namespace cost_functors {
/** \brief Functor to calculate the cost of photometric bundle adjustment between two frames
 * for given landmark
 *
 */
template <energy::motion::Motion Motion, int PatternSize, int C>
class BundleAdjustmentPhotometricCostFunctorAnalytic
    : public ceres::SizedCostFunction<PatternSize * C, Motion::DoF + 2, Motion::DoF + 2, 1> {
 public:
  /** return type of a cost functor */
  enum struct ResidualStatus : uint8_t { kOk, kOOB };

  /** number of residuals */
  static constexpr int residuals_num = PatternSize * C;
  /**
   * Create functor to calculate the cost of photometric bundle adjustment between two frames
   * @param residual_point reference to the residual point in the storage
   * */
  BundleAdjustmentPhotometricCostFunctorAnalytic(const ResidualPoint<double, Motion, PatternSize, C> &residual_point)
      : residual_point_(residual_point) {}
  /**
   * for more information refer to ceres::SizedCostFunction::Evaluate
   *
   * @param[out] residuals_ptr array of pointers to evaluated state
   * @param jacobians array of pointers to evaluated state
   *
   * @return true
   */
  bool Evaluate(double const *const *, double *residuals_ptr, double **jacobians) const final {
    Eigen::Map<Eigen::Matrix<double, residuals_num, 1>> residuals(residuals_ptr);

    memcpy(residuals_ptr, residual_point_.residuals.data(),
           residual_point_.residuals.SizeAtCompileTime * sizeof(double));

    if (jacobians == nullptr) {
      return true;
    }
    if (jacobians[0] != nullptr) {
      memcpy(jacobians[0], residual_point_.d_reference_state_eps.data(),
             residual_point_.d_reference_state_eps.SizeAtCompileTime * sizeof(double));
    }
    if (jacobians[1] != nullptr) {
      memcpy(jacobians[1], residual_point_.d_target_state_eps.data(),
             residual_point_.d_target_state_eps.SizeAtCompileTime * sizeof(double));
    }
    if (jacobians[2] != nullptr) {
      memcpy(jacobians[2], residual_point_.d_idepth.data(),
             residual_point_.d_idepth.SizeAtCompileTime * sizeof(double));
    }
    return true;
  }

 private:
  /** reference to the residual point in the storage */
  const ResidualPoint<double, Motion, PatternSize, C> &residual_point_;
};
}  // namespace cost_functors
}  // namespace problem
}  // namespace energy
}  // namespace dsopp

#endif  // DSOPP_BUNDLE_ADJUSTMENT_PHOTOMETRIC_COST_FUNCTOR_ANALYTIC_HPP
