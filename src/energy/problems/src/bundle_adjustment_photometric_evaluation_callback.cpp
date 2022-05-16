#include "energy/problems/photometric_bundle_adjustment/bundle_adjustment_photometric_evaluation_callback.hpp"

#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/camera_model/pinhole/simple_radial.hpp"

#include "energy/problems/photometric_bundle_adjustment/evaluate_jacobians.hpp"
#include "energy/problems/photometric_bundle_adjustment/local_frame.hpp"
#include "features/camera/pattern_patch.hpp"

namespace dsopp {
namespace energy {
namespace problem {

template <typename Scalar, energy::motion::Motion Motion, typename Model, int PatternSize,
          template <int> typename Grid2D, int C, bool FIRST_ESTIMATE_JACOBIANS, bool OPTIMIZE_IDEPTHS>
BundleAdjustmentPhotometricEvaluationCallback<Scalar, Motion, Model, PatternSize, Grid2D, C, FIRST_ESTIMATE_JACOBIANS,
                                              OPTIMIZE_IDEPTHS>::
    BundleAdjustmentPhotometricEvaluationCallback(
        std::deque<std::unique_ptr<LocalFrame<Scalar, Motion, Model, PatternSize, Grid2D, C>>> &frames)
    : frames_(frames) {}

template <typename Scalar, energy::motion::Motion Motion, typename Model, int PatternSize,
          template <int> typename Grid2D, int C, bool FIRST_ESTIMATE_JACOBIANS, bool OPTIMIZE_IDEPTHS>
void BundleAdjustmentPhotometricEvaluationCallback<Scalar, Motion, Model, PatternSize, Grid2D, C,
                                                   FIRST_ESTIMATE_JACOBIANS,
                                                   OPTIMIZE_IDEPTHS>::PrepareForEvaluation(bool evaluate_jacobians,
                                                                                           bool new_evaluation_point) {
  if (evaluate_jacobians and new_evaluation_point) {
    evaluateJacobians<Scalar, Motion, Model, PatternSize, Grid2D, C, FIRST_ESTIMATE_JACOBIANS, OPTIMIZE_IDEPTHS, true,
                      true>(frames_);
  }
  if (evaluate_jacobians and not new_evaluation_point) {
    evaluateJacobians<Scalar, Motion, Model, PatternSize, Grid2D, C, FIRST_ESTIMATE_JACOBIANS, OPTIMIZE_IDEPTHS, true,
                      false>(frames_);
  }
  if (not evaluate_jacobians and new_evaluation_point) {
    evaluateJacobians<Scalar, Motion, Model, PatternSize, Grid2D, C, FIRST_ESTIMATE_JACOBIANS, OPTIMIZE_IDEPTHS, false,
                      true>(frames_);
  }
}

#define PBAEvaluationCallbackInstantiation(Motion, Model, PatchSize, Grid, C, FIRST_ESTIMATE_JACOBIANS,         \
                                           OPTIMIZE_IDEPTHS)                                                    \
  template class BundleAdjustmentPhotometricEvaluationCallback<                                                 \
      double, energy::motion::Motion<Precision>, energy::model::Model<Precision>, PatchSize, features::Grid, C, \
      FIRST_ESTIMATE_JACOBIANS, OPTIMIZE_IDEPTHS>
PBAEvaluationCallbackInstantiation(SE3, PinholeCamera, Pattern::kSize, PixelMap, 1, true, true);
PBAEvaluationCallbackInstantiation(SE3, PinholeCamera, Pattern::kSize, PixelMap, 1, false, true);
PBAEvaluationCallbackInstantiation(SE3, PinholeCamera, Pattern::kSize, PixelMap, 1, false, false);
PBAEvaluationCallbackInstantiation(SE3, PinholeCamera, Pattern::kSize, CeresGrid, 1, false, false);
PBAEvaluationCallbackInstantiation(SE3, PinholeCamera, 1, PixelMap, 1, false, false);
PBAEvaluationCallbackInstantiation(SE3, SimpleRadialCamera, Pattern::kSize, PixelMap, 1, false, true);
PBAEvaluationCallbackInstantiation(SE3, SimpleRadialCamera, Pattern::kSize, PixelMap, 1, true, true);
PBAEvaluationCallbackInstantiation(SE3, SimpleRadialCamera, Pattern::kSize, PixelMap, 8, true, true);
#undef PBAEvaluationCallbackInstantiation

}  // namespace problem
}  // namespace energy
}  // namespace dsopp
