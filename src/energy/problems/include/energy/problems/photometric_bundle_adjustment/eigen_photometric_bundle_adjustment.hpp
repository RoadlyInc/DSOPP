#ifndef DSOPP_EIGEN_PHOTOMETRIC_BUNDLE_ADJUSTMENT_HPP
#define DSOPP_EIGEN_PHOTOMETRIC_BUNDLE_ADJUSTMENT_HPP

#include "common/pattern/pattern.hpp"
#include "energy/motion/motion.hpp"
#include "energy/normal_linear_system.hpp"
#include "energy/problems/photometric_bundle_adjustment/photometric_bundle_adjustment.hpp"
#include "energy/problems/photometric_bundle_adjustment/trust_region_photometric_bundle_adjustment_options.hpp"

namespace dsopp {
namespace energy {
namespace problem {
/** \brief Class that solve the photometric bundle adjustment using eigen optimizer.
 *
 * This solver can optimize poses and landmarks idepths together or separately.
 *
 * @tparam Calibration type of the calibration to reproject landmarks.
 * @tparam USE_CERES_INTERPOLATOR true for using ceres interpolation in the optimization.
 * @tparam OPTIMIZE_POSES true to optimize poses
 * @tparam OPTIMIZE_IDEPTHS true to optimize idepths
 * @tparam C number of channels in the grid./
 */
template <energy::motion::Motion Motion, model::Model Model, int PatternSize = Pattern::kSize,
          template <int> typename Grid2D = features::PixelMap, bool OPTIMIZE_POSES = true,
          bool OPTIMIZE_IDEPTHS = false, bool FIRST_ESTIMATE_JACOBIANS = OPTIMIZE_IDEPTHS, int C = 1>
class EigenPhotometricBundleAdjustment
    : public PhotometricBundleAdjustment<Precision, Motion, Model, PatternSize, Grid2D, OPTIMIZE_POSES,
                                         OPTIMIZE_IDEPTHS, FIRST_ESTIMATE_JACOBIANS, C> {
 public:
  /**
   * Constructor to create CeresPhotometricBundleAdjustment solver
   * @param trust_region_options trust regeion options
   * @param estimate_uncertainty estimate uncertainty of poses and idepths after optimization
   * @param force_accept accept each iteration of LM, usefull when the initial guess is accurate enough
   */
  EigenPhotometricBundleAdjustment(const TrustRegionPhotometricBundleAdjustmentOptions<Precision> &trust_region_options,
                                   bool estimate_uncertainty = false, bool force_accept = false);
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

  ~EigenPhotometricBundleAdjustment();
  /**
   * Push a new frame in the solver. The new frame should have a later timestamp than the previous frames.
   * A local copy of the frame will be created for optimization, the frame will not be changed.
   *
   * @tparam FrameType frame type can be immature, active or marginalized.
   * @param frame new frame to use in optimization
   * @param level level of the pyramid which will be added to the solver
   * @param model camera model for the frame
   * @param frame_parameterization constraint on frame position
   */
  void pushFrame(const track::ActiveKeyframe<Motion> &frame, size_t level, const Model &model,
                 FrameParameterization frame_parameterization = FrameParameterization::kFree);

 private:
  /** marginalized part of the problem */
  NormalLinearSystem<double> system_marginalized_{0};
  /** energy at the marginalization point */
  Precision energy_marginalized_ = 0;
  /**  accept each iteration of LM, usefull when the initial guess is accurate enough */
  bool force_accept_;
  /** trust region optimization options */
  TrustRegionPhotometricBundleAdjustmentOptions<Precision> trust_region_options_;
  /** the total number of frames that have been optimized */
};
}  // namespace problem
}  // namespace energy
}  // namespace dsopp

#endif  // DSOPP_EIGEN_PHOTOMETRIC_BUNDLE_ADJUSTMENT_HPP
