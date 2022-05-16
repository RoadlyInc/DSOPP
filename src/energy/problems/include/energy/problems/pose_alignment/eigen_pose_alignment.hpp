#ifndef DSOPP_EIGEN_POSE_ALIGNMENT_HPP
#define DSOPP_EIGEN_POSE_ALIGNMENT_HPP

#include "energy/problems/pose_alignment/pose_alignment.hpp"

#include <optional>

#include "common/settings.hpp"
#include "energy/motion/motion.hpp"
#include "energy/problems/photometric_bundle_adjustment/trust_region_photometric_bundle_adjustment_options.hpp"
#include "energy/problems/pose_alignment/pose_alignment.hpp"

namespace dsopp {
namespace energy {
namespace problem {
/** \brief Class that solve the photometric bundle adjustment for 2 frames using custom solver.
 *
 * @tparam Motion motion type
 * @tparam Model camera model type
 * @tparam PatternSize pattern size used in the alignment
 * @tparam Grid2D jacobian or intensities interpolator
 * @tparam C number of channels in the grid
 */
template <energy::motion::Motion Motion, model::Model Model, int PatternSize = 1,
          template <int> typename Grid2D = features::PixelMap, int C = 1, bool OPTIMIZE_AFFINE_BRIGHTNESS = true>
class EigenPoseAlignment : public PoseAlignment<Motion, Model, PatternSize, Grid2D, C> {
 public:
  /**
   * Constructor to create CeresPhotometricBundleAdjustment solver
   * @param trust_region_options trust regeion options
   */
  EigenPoseAlignment(const TrustRegionPhotometricBundleAdjustmentOptions<Precision> &trust_region_options);

  /**
   * Solve the photometric bundle adjustment problem.
   *
   * @param number_of_threads number of threads for optimizer
   * @return return root mean square error in energy per pixel
   */
  Precision solve(const size_t number_of_threads);

  /**
   * @return covariance for relative pose t_t_r
   */
  Eigen::Matrix<Precision, Motion::DoF, Motion::DoF> tTargetReferenceCovariance() const;

  /**
   * set rotation prior ``rotation_prior_``
   * @param r_t_r rotation prior for relative pose
   */
  void setRotationPrior(const Eigen::Matrix3<Precision> &r_t_r);

  /**
   * clears solver, deletes all frames and poses
   */
  void reset();

  void pushKnownPose(time timestamp, const Motion &t_w_agent);

  ~EigenPoseAlignment();

 private:
  /** rotation prior for relative pose */
  std::optional<Eigen::Matrix3<Precision>> prior_rotation_t_r_;
  /** trust region optimization options */
  TrustRegionPhotometricBundleAdjustmentOptions<Precision> trust_region_options_;
  /** covariance for relative pose t_t_r */
  Eigen::Matrix<Precision, Motion::DoF, Motion::DoF> covariance_t_t_r_;
};
}  // namespace problem
}  // namespace energy
}  // namespace dsopp

#endif  // DSOPP_EIGEN_POSE_ALIGNMENT_HPP
