
#ifndef DSOPP_CERES_POSE_ALIGNMENT_SOLVER_HPP
#define DSOPP_CERES_POSE_ALIGNMENT_SOLVER_HPP

#include "common/pattern/pattern.hpp"
#include "energy/motion/motion.hpp"
#include "energy/problems/photometric_bundle_adjustment/photometric_bundle_adjustment.hpp"
#include "energy/problems/photometric_bundle_adjustment/trust_region_photometric_bundle_adjustment_options.hpp"

namespace dsopp {
namespace energy {
namespace problem {
/** \brief Class that solve the photometric bundle adjustment using Ceres optimizer.
 *
 * This solver can optimize poses and landmarks idepths together or separately.
 *
 * @tparam Calibration type of the calibration to reproject landmarks.
 * @tparam USE_CERES_INTERPOLATOR true for using ceres interpolation in the optimization.
 * @tparam OPTIMIZE_POSES true to optimize poses
 * @tparam OPTIMIZE_IDEPTHS true to optimize idepths
 * @tparam C number of channels in the grid
 */
template <energy::motion::Motion Motion, model::Model Model, int PatternSize = Pattern::kSize,
          template <int> typename Grid2D = features::PixelMap, bool OPTIMIZE_POSES = true,
          bool OPTIMIZE_IDEPTHS = false, bool FIRST_ESTIMATE_JACOBIANS = OPTIMIZE_IDEPTHS, bool OPTIMIZE_CAMERA = false,
          int C = 1>
class CeresPhotometricBundleAdjustment
    : public PhotometricBundleAdjustment<double, Motion, Model, PatternSize, Grid2D, OPTIMIZE_POSES, OPTIMIZE_IDEPTHS,
                                         FIRST_ESTIMATE_JACOBIANS, C> {
 public:
  /**
   * Constructor to create CeresPhotometricBundleAdjustment solver
   * @param trust_region_options trust regeion options
   * @param estimate_uncertainty estimate of uncertainty od poses and idepths after optimization
   * @param use_analytic_diff use analytic derivatives
   * @param level pyramid level of optimization
   */
  CeresPhotometricBundleAdjustment(const TrustRegionPhotometricBundleAdjustmentOptions<double> &trust_region_options,
                                   bool estimate_uncertainty = false, bool use_analytic_diff = true, size_t level = 0);
  /**
   * Solve the photometric bundle adjustment broblem.
   *
   * @param number_of_threads number of threads for ceres optimizer
   * @return final cost after optimization
   */
  Precision solve(const size_t number_of_threads);
  /**
   * update information in local frame
   *
   * @param frame frame to get updates from
   */
  void updateLocalFrame(const track::ActiveKeyframe<Motion> &frame);
  /**
   * create camera model from the optimized intrinsic parameters
   * @param frame_id id of a frame
   * @return optimized camera model
   */
  Model optimizedCameraModel(size_t frame_id) const;

  ~CeresPhotometricBundleAdjustment();

 private:
  /** evaluation callback */
  std::unique_ptr<BundleAdjustmentPhotometricEvaluationCallback<double, Motion, Model, PatternSize, Grid2D, C,
                                                                FIRST_ESTIMATE_JACOBIANS, OPTIMIZE_IDEPTHS>>
      evaluation_callback_;
  /** flag for usage of analytical derivatives (wouldn't work with camera optimization) */
  bool use_analytic_diff_;
  /** pyramid level of optimization */
  size_t level_;
  /** trust region optimization options */
  TrustRegionPhotometricBundleAdjustmentOptions<double> trust_region_options_;
};
}  // namespace problem
}  // namespace energy
}  // namespace dsopp

#endif  // DSOPP_CERES_POSE_ALIGNMENT_SOLVER_HPP
