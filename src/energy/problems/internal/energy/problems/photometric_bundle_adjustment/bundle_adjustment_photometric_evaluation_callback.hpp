#ifndef DSOPP_BUNDLE_ADJUSTMENT_PHOTOMETRIC_EVALUATION_CALLBACK_HPP
#define DSOPP_BUNDLE_ADJUSTMENT_PHOTOMETRIC_EVALUATION_CALLBACK_HPP

#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"
#include "sensors/camera_calibration/mask/camera_mask.hpp"

#include <ceres/ceres.h>
#include <deque>

namespace dsopp {
namespace features {
class PatternPatch;
}  // namespace features
namespace energy {
namespace problem {
template <typename Scalar, energy::motion::Motion Motion, energy::model::Model Model, int PatternSize,
          template <int> typename Grid2D, int C>
class LocalFrame;
/** \brief Using this callback interface, Ceres can notify you when it is
 * about to evaluate the residuals or jacobians.
 *
 * @tparam Motion Motion type
 * @tparam Model type of the model to reproject landmarks.
 * @tparam Grid2D 2D grid to evaluate data
 * @tparam N size of the patch.
 * @tparam OPTIMIZE_IDEPTHS true to optimize idepths
 */
template <typename Scalar, energy::motion::Motion Motion, typename Model, int PatternSize,
          template <int> typename Grid2D, int C, bool FIRST_ESTIMATE_JACOBIANS, bool OPTIMIZE_IDEPTHS>
class BundleAdjustmentPhotometricEvaluationCallback : public ceres::EvaluationCallback {
 public:
  /**
   *
   * Creates callback interface with given residuals storage.
   *
   * @param frames local frames in the problem
   *
   */
  BundleAdjustmentPhotometricEvaluationCallback(
      std::deque<std::unique_ptr<LocalFrame<Scalar, Motion, Model, PatternSize, Grid2D, C>>> &frames);
  /**
   * Notification before the ceres evaluation.
   *
   * @param evaluate_jacobians true if it is necessary to evaluate with jacobians and false if without
   * jacobians.
   * @param new_evaluation_point true if it is necessary to evaluate residuals and false if without
   * residuals.
   * */
  void PrepareForEvaluation(bool evaluate_jacobians, bool new_evaluation_point) final;

  virtual ~BundleAdjustmentPhotometricEvaluationCallback() = default;

 private:
  /** reference to the residuals storage */
  std::deque<std::unique_ptr<LocalFrame<Scalar, Motion, Model, PatternSize, Grid2D, C>>> &frames_;
};
}  // namespace problem
}  // namespace energy
}  // namespace dsopp

#endif  // DSOPP_BUNDLE_ADJUSTMENT_PHOTOMETRIC_EVALUATION_CALLBACK_HPP
