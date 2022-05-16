#include "energy/problems/photometric_bundle_adjustment/eigen_photometric_bundle_adjustment.hpp"

#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/camera_model/pinhole/simple_radial.hpp"
#include "energy/levenberg_marquardt_algorithm/levenberg_marquardt_algorithm.hpp"

#include "energy/normal_linear_system.hpp"
#include "energy/problems/cost_functors/affine_brightness_regularization_cost_functor.hpp"
#include "energy/problems/cost_functors/bundle_adjustment_photometric_cost_functor.hpp"
#include "energy/problems/cost_functors/bundle_adjustment_photometric_cost_functor_analytic.hpp"
#include "energy/problems/depth_map.hpp"

#include "energy/problems/photometric_bundle_adjustment/covariance_matrices_of_relative_poses.hpp"
#include "energy/problems/photometric_bundle_adjustment/eigen_photometric_bundle_adjustment_problem.hpp"
#include "energy/problems/photometric_bundle_adjustment/evaluate_jacobians.hpp"
#include "energy/problems/photometric_bundle_adjustment/hessian_block_evaluation.hpp"
#include "energy/problems/photometric_bundle_adjustment/state_priors.hpp"
#include "features/camera/ceres_grid.hpp"
#include "features/camera/pixel_map.hpp"

#include "track/connections/frame_connection.hpp"
#include "track/frames/active_keyframe.hpp"
#include "track/landmarks/active_tracking_landmark.hpp"
#include "track/landmarks/immature_tracking_landmark.hpp"
#include "track/landmarks/tracking_landmark.hpp"

namespace dsopp {
namespace energy {
namespace problem {

Eigen::MatrixX<Precision> pseudoInverse(const Eigen::MatrixX<Precision> &origin, const int number_of_nullspaces) {
  Eigen::JacobiSVD<Eigen::MatrixX<Precision>> svd_holder(origin, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::MatrixX<Precision> U = svd_holder.matrixU();
  Eigen::MatrixX<Precision> V = svd_holder.matrixV();
  Eigen::MatrixX<Precision> D = svd_holder.singularValues();

  Eigen::MatrixX<Precision> S(V.cols(), U.cols());
  S.setZero();

  for (int i = 0; i < D.size() - number_of_nullspaces; ++i) {
    S(i, i) = 1 / D(i, 0);
  }

  return V * S * U.transpose();
}

template <energy::motion::Motion Motion, model::Model Model, int PatternSize, template <int> typename Grid2D,
          bool OPTIMIZE_POSES, bool OPTIMIZE_IDEPTHS, bool FIRST_ESTIMATE_JACOBIANS, int C>
EigenPhotometricBundleAdjustment<
    Motion, Model, PatternSize, Grid2D, OPTIMIZE_POSES, OPTIMIZE_IDEPTHS, FIRST_ESTIMATE_JACOBIANS,
    C>::EigenPhotometricBundleAdjustment(const TrustRegionPhotometricBundleAdjustmentOptions<Precision>
                                             &trust_region_options,
                                         bool estimate_uncertainty, bool force_accept)
    : PhotometricBundleAdjustment<Precision, Motion, Model, PatternSize, Grid2D, OPTIMIZE_POSES, OPTIMIZE_IDEPTHS,
                                  FIRST_ESTIMATE_JACOBIANS, C>(estimate_uncertainty),
      force_accept_(force_accept),
      trust_region_options_(trust_region_options) {}

template <energy::motion::Motion Motion, model::Model Model, int PatternSize, template <int> typename Grid2D,
          bool OPTIMIZE_POSES, bool OPTIMIZE_IDEPTHS, bool FIRST_ESTIMATE_JACOBIANS, int C>
Precision EigenPhotometricBundleAdjustment<Motion, Model, PatternSize, Grid2D, OPTIMIZE_POSES, OPTIMIZE_IDEPTHS,
                                           FIRST_ESTIMATE_JACOBIANS, C>::solve(const size_t number_of_threads) {
  (void)number_of_threads;
  CHECK(this->frames_[0]->sensors().size() == 1);
  size_t sensor_id = this->frames_[0]->sensors()[0];

  energy::levenberg_marquardt_algorithm::Options options;
  options.initial_levenberg_marquardt_regularizer = 1.0_p / trust_region_options_.initial_trust_region_radius;
  options.function_tolerance = trust_region_options_.function_tolerance;
  options.parameter_tolerance = trust_region_options_.parameter_tolerance;
  options.max_num_iterations = trust_region_options_.max_iterations;
  options.min_num_iterations = 3;
  options.force_accept = force_accept_;
  options.levenberg_marquardt_regularizer_decrease_on_accept = 1.;
  options.levenberg_marquardt_regularizer_increase_on_reject = 1.;

  PhotometricBundleAdjustmentProblem<Motion, Model, PatternSize, Grid2D, OPTIMIZE_POSES, OPTIMIZE_IDEPTHS,
                                     FIRST_ESTIMATE_JACOBIANS, C>
      problem(this->frames_, sensor_id, trust_region_options_.sigma_huber_loss, system_marginalized_.cast<Precision>(),
              energy_marginalized_, trust_region_options_.affine_brightness_regularizer,
              trust_region_options_.fixed_state_regularizer);

  if constexpr (FIRST_ESTIMATE_JACOBIANS) {
    this->firstEstimateJacobians();
  }
  auto result = energy::levenberg_marquardt_algorithm::solve(problem, options);

  const size_t minimum_valid_reprojections_num = 1;

  this->relinearizeSystem();
  if (this->estimate_uncertainty_) {
    this->firstEstimateJacobians();
    covarianceMatricesOfRelativePoses(
        this->frames_,
        covarianceMatrixPosePose<Motion, Model, PatternSize, Grid2D, OPTIMIZE_IDEPTHS, FIRST_ESTIMATE_JACOBIANS, C>(
            this->frames_, sensor_id, this->system_marginalized_, trust_region_options_.affine_brightness_regularizer,
            trust_region_options_.fixed_state_regularizer));
  }
  this->updatePointStatuses(minimum_valid_reprojections_num, trust_region_options_.sigma_huber_loss);
  return result.energy;
}

template <energy::motion::Motion Motion, model::Model Model, int PatternSize, template <int> typename Grid2D,
          bool OPTIMIZE_POSES, bool OPTIMIZE_IDEPTHS, bool FIRST_ESTIMATE_JACOBIANS, int C>
void EigenPhotometricBundleAdjustment<Motion, Model, PatternSize, Grid2D, OPTIMIZE_POSES, OPTIMIZE_IDEPTHS,
                                      FIRST_ESTIMATE_JACOBIANS, C>::updateLocalFrame(const track::ActiveKeyframe<Motion>
                                                                                         &frame) {
  auto local_frame = this->getLocalFrame(frame.timestamp());
  CHECK(local_frame) << "Cannot update frame, there is no local copy in the solver";
  local_frame->update(frame, frame.connections());
  local_frame->to_marginalize = frame.isMarginalized() && !local_frame->is_marginalized;
  local_frame->is_marginalized = frame.isMarginalized();
}

template <energy::motion::Motion Motion, model::Model Model, int PatternSize, template <int> typename Grid2D,
          bool OPTIMIZE_POSES, bool OPTIMIZE_IDEPTHS, bool FIRST_ESTIMATE_JACOBIANS, int C>
void EigenPhotometricBundleAdjustment<Motion, Model, PatternSize, Grid2D, OPTIMIZE_POSES, OPTIMIZE_IDEPTHS,
                                      FIRST_ESTIMATE_JACOBIANS,
                                      C>::pushFrame(const track::ActiveKeyframe<Motion> &frame, size_t level,
                                                    const Model &model, FrameParameterization frame_parameterization) {
  if (this->frames_.size() > 1) {
    size_t sensor_id = this->frames_[0]->sensors()[0];
    this->firstEstimateJacobians();
    evaluateJacobians<Precision, Motion, Model, PatternSize, Grid2D, C, FIRST_ESTIMATE_JACOBIANS, OPTIMIZE_IDEPTHS,
                      true, true, true>(this->frames_, trust_region_options_.sigma_huber_loss);
    changeResidualStatuses(this->frames_);
    updateMarginalizedLinearSystem(this->frames_, sensor_id, system_marginalized_, energy_marginalized_,
                                   trust_region_options_.affine_brightness_regularizer,
                                   trust_region_options_.fixed_state_regularizer);
  }
  this->PhotometricBundleAdjustment<Precision, Motion, Model, PatternSize, Grid2D, OPTIMIZE_POSES, OPTIMIZE_IDEPTHS,
                                    FIRST_ESTIMATE_JACOBIANS, C>::pushFrame(frame, level, model,
                                                                            frame_parameterization);
  const int kBlockSize = Motion::DoF + 2;
  long new_size = kBlockSize * static_cast<long>(this->frames_.size());
  long new_parameters_num = new_size - system_marginalized_.H.rows();
  system_marginalized_.resize(new_size);
  system_marginalized_.H.rightCols(new_parameters_num).setZero();
  system_marginalized_.H.bottomRows(new_parameters_num).setZero();
  system_marginalized_.b.tail(new_parameters_num).setZero();
}

template <energy::motion::Motion Motion, model::Model Model, int PatternSize, template <int> typename Grid2D,
          bool OPTIMIZE_POSES, bool OPTIMIZE_IDEPTHS, bool FIRST_ESTIMATE_JACOBIANS, int C>
EigenPhotometricBundleAdjustment<Motion, Model, PatternSize, Grid2D, OPTIMIZE_POSES, OPTIMIZE_IDEPTHS,
                                 FIRST_ESTIMATE_JACOBIANS, C>::~EigenPhotometricBundleAdjustment() = default;
/// \cond DO_NOT_DOCUMENT

#define PBAEigenInstantiation(Motion, Model, PatchSize, Grid, OPTIMIZE_POSES, OPTIMIZE_IDEPTHS,                \
                              FIRST_ESTIMATE_JACOBIANS, C)                                                     \
  template class EigenPhotometricBundleAdjustment<energy::motion::Motion<Precision>, model::Model<Precision>,  \
                                                  PatchSize, features::Grid, OPTIMIZE_POSES, OPTIMIZE_IDEPTHS, \
                                                  FIRST_ESTIMATE_JACOBIANS, C>

PBAEigenInstantiation(SE3, PinholeCamera, Pattern::kSize, PixelMap, true, true, true, 1);
PBAEigenInstantiation(SE3, PinholeCamera, Pattern::kSize, PixelMap, true, false, false, 1);
PBAEigenInstantiation(SE3, SimpleRadialCamera, Pattern::kSize, PixelMap, true, true, true, 1);

#undef PBAEigenInstantiation

/// \endcond
}  // namespace problem
}  // namespace energy
}  // namespace dsopp
