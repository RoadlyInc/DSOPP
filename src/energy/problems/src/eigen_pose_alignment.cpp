#include "energy/problems/pose_alignment/eigen_pose_alignment.hpp"

#include <Eigen/Dense>
#include <chrono>

#include "common/settings.hpp"
#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/camera_model/pinhole/simple_radial.hpp"
#include "energy/levenberg_marquardt_algorithm/levenberg_marquardt_algorithm.hpp"
#include "energy/motion/se3_motion.hpp"

#include "energy/normal_linear_system.hpp"
#include "energy/problems/photometric_bundle_adjustment/local_frame.hpp"
#include "energy/problems/photometric_bundle_adjustment/state_priors.hpp"
#include "energy/projector/camera_reproject.hpp"
#include "features/camera/ceres_grid.hpp"
#include "features/camera/pixel_map.hpp"
#include "measures/similarity_measure_ssd.hpp"
#include "sensors/camera_calibration/mask/camera_mask.hpp"

namespace dsopp {
namespace energy {
namespace problem {

namespace {
template <energy::motion::Motion Motion, model::Model Model, int PatternSize, template <int> typename Grid2D, int C,
          bool OPTIMIZE_AFFINE_BRIGHTNESS>
class PoseAlignerProblem {
 public:
  static constexpr int kNumParameters = Motion::DoF + (OPTIMIZE_AFFINE_BRIGHTNESS ? 2 : 0);

  PoseAlignerProblem(const LocalFrame<Precision, Motion, Model, PatternSize, Grid2D, C> &reference_frame,
                     const LocalFrame<Precision, Motion, Model, PatternSize, Grid2D, C> &target_frame,
                     const size_t sensor_id, const sensors::calibration::CameraMask &mask,
                     const Precision sigma_huber_loss, const Eigen::Vector2<Precision> &affine_brightness_regularizer,
                     typename Motion::Product &t_t_r, Eigen::Vector2<Precision> &affine_brightness_eps)
      : reference_frame_(reference_frame),
        target_frame_(target_frame),
        sensor_id_(sensor_id),
        mask_(mask),
        sigma_huber_loss_(sigma_huber_loss),
        affine_brightness_regularizer_(affine_brightness_regularizer),
        t_t_r_(t_t_r),
        affine_brightness_eps_(affine_brightness_eps),
        success_statuses_(reference_frame_.active_landmarks.at(sensor_id_).size(), false),
        target_patches_(reference_frame_.active_landmarks.at(sensor_id_).size()),
        d_intensity_u_diags_(reference_frame_.active_landmarks.at(sensor_id_).size()),
        d_intensity_v_diags_(reference_frame_.active_landmarks.at(sensor_id_).size()) {
    old_t_t_r_ = t_t_r_;
    old_affine_brightness_eps_ = affine_brightness_eps_;
    system_.setZero();
    step_.setZero();
  }

  std::pair<Precision, int> calculateEnergy() {
    const Precision kSigmaHuberSqr = sigma_huber_loss_ * sigma_huber_loss_;

    Precision energy = 0;
    int number_of_valid_residuals = 0;

    Eigen::Matrix<Precision, 2, PatternSize> target_pattern;
    Eigen::Matrix<Precision, PatternSize, C, PatchStorageOrder<C>> residuals;

    reprojection::ArrayReprojector<Precision, Model, typename Motion::Product> reprojector(reference_frame_.model,
                                                                                           target_frame_.model, t_t_r_);
    const Eigen::Vector2<Precision> &reference_affine_brightness = reference_frame_.affine_brightness0;
    const Eigen::Vector2<Precision> target_affine_brightness =
        target_frame_.affine_brightness0 + affine_brightness_eps_;

    const auto &grid = *target_frame_.grids.at(sensor_id_);

    Precision brightness_change_scale = (target_frame_.exposure_time / reference_frame_.exposure_time) *
                                        std::exp(target_affine_brightness[0] - reference_affine_brightness[0]);
    auto &active_landmarks = reference_frame_.active_landmarks.at(sensor_id_);
    for (size_t i = 0; i < active_landmarks.size(); ++i) {
      auto &landmark = active_landmarks[i];
      bool success = reprojector.reprojectPattern(landmark.reference_pattern, landmark.idepth, target_pattern);
      success = success && mask_.valid(target_pattern);
      success_statuses_[i] = success;
      if (success) {
        grid.Evaluate(target_pattern, target_patches_[i], d_intensity_u_diags_[i], d_intensity_v_diags_[i]);
        measure::SimilarityMeasureSSD::residuals(
            (target_patches_[i] -
             Eigen::Matrix<Precision, PatternSize, C, PatchStorageOrder<C>>::Constant(target_affine_brightness[1])),
            brightness_change_scale *
                (landmark.patch - Eigen::Matrix<Precision, PatternSize, C, PatchStorageOrder<C>>::Constant(
                                      reference_affine_brightness[1])),
            residuals);
        Precision residuals_norm = residuals.norm();
        Precision residuals_squared_norm = residuals_norm * residuals_norm;
        bool huber_linear = residuals_squared_norm > kSigmaHuberSqr;
        if (huber_linear) {
          energy += sigma_huber_loss_ * residuals_norm - kSigmaHuberSqr / 2;
        } else {
          energy += residuals_squared_norm / 2;
        }
        number_of_valid_residuals++;
      }
    }

    if constexpr (OPTIMIZE_AFFINE_BRIGHTNESS) {
      energy +=
          hessian_prior::AffineBrightnessPrior::energyTerm(target_affine_brightness, affine_brightness_regularizer_);
    }
    energy += hessian_prior::MotionPrior<Motion>::energyTerm(t_t_r_.inverse(), &(reference_frame_.model),
                                                             target_frame_.timestamp - reference_frame_.timestamp);
    return {energy, number_of_valid_residuals};
  }

  void linearize() {
    const Precision kSigmaHuberSqr = sigma_huber_loss_ * sigma_huber_loss_;

    system_.setZero();

    Eigen::Matrix<Precision, 2, PatternSize> target_pattern;
    Eigen::Matrix<Precision, PatternSize, C, PatchStorageOrder<C>> residuals;
    Eigen::Vector<Precision, PatternSize> d_u_idepth;
    Eigen::Vector<Precision, PatternSize> d_v_idepth;
    Eigen::Matrix<Precision, PatternSize, Motion::Product::DoF> d_v_tTargetReference;
    Eigen::Matrix<Precision, PatternSize, Motion::Product::DoF> d_u_tTargetReference;
    Eigen::Matrix<Precision, PatternSize, C, PatchStorageOrder<C>> target_patch;
    Eigen::Vector<Precision, PatternSize * C> d_intensity_u_diag;
    Eigen::Vector<Precision, PatternSize * C> d_intensity_v_diag;
    Eigen::Matrix<Precision, PatternSize * C, kNumParameters> d_state;
    d_state.setZero();
    if constexpr (OPTIMIZE_AFFINE_BRIGHTNESS) {
      d_state.col(Sophus::SE3d::DoF + 1).setConstant(-1);
    }

    reprojection::ArrayReprojector<Precision, Model, typename Motion::Product, false> reprojector(
        reference_frame_.model, target_frame_.model, t_t_r_);
    const Eigen::Vector2<Precision> &reference_affine_brightness = reference_frame_.affine_brightness0;
    const Eigen::Vector2<Precision> target_affine_brightness =
        target_frame_.affine_brightness0 + affine_brightness_eps_;

    Precision brightness_change_scale = (target_frame_.exposure_time / reference_frame_.exposure_time) *
                                        std::exp(target_affine_brightness[0] - reference_affine_brightness[0]);
    const auto &landmarks = reference_frame_.active_landmarks.at(sensor_id_);

    auto logTransformer = t_t_r_.leftLogTransformer();

    for (size_t landmark_i = 0; landmark_i < landmarks.size(); ++landmark_i) {
      const auto &landmark = landmarks[landmark_i];
      reprojector.reprojectPattern(landmark.reference_pattern, landmark.idepth, target_pattern, d_u_idepth, d_v_idepth,
                                   d_u_tTargetReference, d_v_tTargetReference);
      if (success_statuses_[landmark_i]) {
        const Eigen::Matrix<Precision, PatternSize, C, PatchStorageOrder<C>> residuals_left =
            target_patches_[landmark_i] -
            Eigen::Matrix<Precision, PatternSize, C, PatchStorageOrder<C>>::Constant(target_affine_brightness[1]);
        const Eigen::Matrix<Precision, PatternSize, C, PatchStorageOrder<C>> residuals_right =
            brightness_change_scale *
            (landmark.patch -
             Eigen::Matrix<Precision, PatternSize, C, PatchStorageOrder<C>>::Constant(reference_affine_brightness[1]));
        measure::SimilarityMeasureSSD::residuals(residuals_left, residuals_right, residuals);
        bool huber_linear = residuals.squaredNorm() > kSigmaHuberSqr;
        Precision huber_weight = huber_linear ? sigma_huber_loss_ / residuals.norm() : 1;

        for (size_t i = 0; i < PatternSize; ++i) {
          d_state.template block<C, Motion::DoF>(static_cast<int>(i * C), 0).noalias() =
              -(d_intensity_u_diags_[landmark_i].template segment<C>(static_cast<int>(i * C)) *
                    d_u_tTargetReference.row(static_cast<int>(i)) +
                d_intensity_v_diags_[landmark_i].template segment<C>(static_cast<int>(i * C)) *
                    d_v_tTargetReference.row(static_cast<int>(i))) *
              logTransformer;
        }

        if constexpr (OPTIMIZE_AFFINE_BRIGHTNESS) {
          for (size_t i = 0; i < PatternSize; ++i) {
            d_state.col(Motion::DoF).template segment<C>(static_cast<int>(i * C)).noalias() =
                -residuals_right.row(static_cast<int>(i));
          }
        }

        system_.H.noalias() += huber_weight * d_state.transpose().lazyProduct(d_state);

        for (size_t i = 0; i < PatternSize; ++i) {
          system_.b.noalias() += huber_weight * d_state.template block<C, kNumParameters>(static_cast<int>(i * C), 0)
                                                    .transpose()
                                                    .lazyProduct(residuals.row(static_cast<int>(i)).transpose());
        }
      }
    }
    if constexpr (OPTIMIZE_AFFINE_BRIGHTNESS) {
      auto affine_brightness_prior =
          hessian_prior::AffineBrightnessPrior::priorSystem(target_affine_brightness, affine_brightness_regularizer_);
      system_.addToBlock(Motion::DoF, affine_brightness_prior);
    }
    auto motion_prior =
        hessian_prior::MotionPrior<Motion>::systemPrior(t_t_r_.inverse(), t_t_r_.inverse(), &reference_frame_.model,
                                                        target_frame_.timestamp - reference_frame_.timestamp);
    system_.addToBlock(0, motion_prior.first);
  }

  void calculateStep(const Precision levenberg_marquardt_regularizer) {
    NormalLinearSystem system_regularized = system_;
    system_regularized.H += (system_.H.diagonal() * levenberg_marquardt_regularizer).asDiagonal();
    step_ = system_regularized.solve();

    old_t_t_r_ = t_t_r_;
    t_t_r_ = t_t_r_.leftIncrement(step_.template head<Motion::DoF>());

    if constexpr (OPTIMIZE_AFFINE_BRIGHTNESS) {
      old_affine_brightness_eps_ = affine_brightness_eps_;
      affine_brightness_eps_ -= step_.template tail<2>();
    }
  }

  std::pair<Precision, Precision> acceptStep() {
    return {OPTIMIZE_AFFINE_BRIGHTNESS ? (target_frame_.affine_brightness0 + old_affine_brightness_eps_).squaredNorm()
                                       : 0_p,
            step_.squaredNorm()};
  }

  void rejectStep() {
    t_t_r_ = old_t_t_r_;
    affine_brightness_eps_ = old_affine_brightness_eps_;
  }

  bool stop() { return false; }

  Eigen::Matrix<Precision, kNumParameters, kNumParameters> hessian() const { return system_.H; }

 private:
  const LocalFrame<Precision, Motion, Model, PatternSize, Grid2D, C> &reference_frame_;
  const LocalFrame<Precision, Motion, Model, PatternSize, Grid2D, C> &target_frame_;
  const size_t sensor_id_;
  const sensors::calibration::CameraMask &mask_;
  const Precision sigma_huber_loss_;
  const Eigen::Vector2<Precision> &affine_brightness_regularizer_;
  typename Motion::Product &t_t_r_;
  Eigen::Vector2<Precision> &affine_brightness_eps_;
  typename Motion::Product old_t_t_r_;
  Eigen::Vector2<Precision> old_affine_brightness_eps_;
  NormalLinearSystem<Precision, kNumParameters> system_;
  Eigen::Vector<Precision, kNumParameters> step_;
  std::vector<bool> success_statuses_;

  std::vector<Eigen::Matrix<Precision, PatternSize, C, PatchStorageOrder<C>>> target_patches_;
  std::vector<Eigen::Vector<Precision, PatternSize * C>> d_intensity_u_diags_;
  std::vector<Eigen::Vector<Precision, PatternSize * C>> d_intensity_v_diags_;
};
}  // namespace

template <energy::motion::Motion Motion, model::Model Model, int PatternSize, template <int> typename Grid2D, int C,
          bool OPTIMIZE_AFFINE_BRIGHTNESS>
EigenPoseAlignment<Motion, Model, PatternSize, Grid2D, C, OPTIMIZE_AFFINE_BRIGHTNESS>::EigenPoseAlignment(
    const TrustRegionPhotometricBundleAdjustmentOptions<Precision> &trust_region_options)
    : trust_region_options_(trust_region_options) {
  this->timestamp_t_w_agents_ = common::timestamp_storage::TimestampStorage<Motion>();
}

template <energy::motion::Motion Motion, model::Model Model, int PatternSize, template <int> typename Grid2D, int C,
          bool OPTIMIZE_AFFINE_BRIGHTNESS>
void EigenPoseAlignment<Motion, Model, PatternSize, Grid2D, C, OPTIMIZE_AFFINE_BRIGHTNESS>::setRotationPrior(
    const Eigen::Matrix3<Precision> &r_t_r) {
  prior_rotation_t_r_ = r_t_r;
}

template <energy::motion::Motion Motion, model::Model Model, int PatternSize, template <int> typename Grid2D, int C,
          bool OPTIMIZE_AFFINE_BRIGHTNESS>
void EigenPoseAlignment<Motion, Model, PatternSize, Grid2D, C, OPTIMIZE_AFFINE_BRIGHTNESS>::reset() {
  prior_rotation_t_r_ = std::nullopt;
  this->frames_.clear();
}

template <energy::motion::Motion Motion, model::Model Model, int PatternSize, template <int> typename Grid2D, int C,
          bool OPTIMIZE_AFFINE_BRIGHTNESS>
void EigenPoseAlignment<Motion, Model, PatternSize, Grid2D, C, OPTIMIZE_AFFINE_BRIGHTNESS>::pushKnownPose(
    time timestamp, const Motion &t_w_agent) {
  this->timestamp_t_w_agents_.pushData(timestamp, t_w_agent);
}

template <energy::motion::Motion Motion, model::Model Model, int PatternSize, template <int> typename Grid2D, int C,
          bool OPTIMIZE_AFFINE_BRIGHTNESS>
Precision EigenPoseAlignment<Motion, Model, PatternSize, Grid2D, C, OPTIMIZE_AFFINE_BRIGHTNESS>::solve(
    const size_t number_of_threads) {
  (void)number_of_threads;
  auto &reference_frame = *this->frames_[0];
  auto &target_frame = *this->frames_.back();

  auto target_found = this->timestamp_t_w_agents_.getData(target_frame.timestamp);

  if (target_found) {
    target_frame.T_w_agent_linearization_point = *target_found;
    target_frame.state_eps.setZero();

    return PoseAlignment<Motion, Model, PatternSize, Grid2D, C>::kZeroCost;
  }

  assert(this->frames_.size() == 2);
  assert(reference_frame.frame_parameterization == FrameParameterization::kFixed);
  assert(target_frame.frame_parameterization == FrameParameterization::kFree);
  assert(reference_frame.sensors().size() == 1);
  size_t sensor_id = reference_frame.sensors()[0];

  const auto &target_mask = target_frame.masks.at(sensor_id);

  energy::levenberg_marquardt_algorithm::Options options;
  options.initial_levenberg_marquardt_regularizer =
      static_cast<Precision>(1. / trust_region_options_.initial_trust_region_radius);
  options.function_tolerance = trust_region_options_.function_tolerance;
  options.parameter_tolerance = trust_region_options_.parameter_tolerance;
  options.max_num_iterations = trust_region_options_.max_iterations;
  options.levenberg_marquardt_regularizer_decrease_on_accept = 2.;
  options.levenberg_marquardt_regularizer_increase_on_reject = 2.;

  typename Motion::Product t_t_r =
      target_frame.T_w_agent_linearization_point.inverse() * reference_frame.T_w_agent_linearization_point;
  if (prior_rotation_t_r_) {
    t_t_r.setRotationMatrix(*prior_rotation_t_r_);
  }
  Eigen::Vector2<Precision> affine_brightness_eps = Eigen::Vector2<Precision>::Zero();

  PoseAlignerProblem<Motion, Model, PatternSize, Grid2D, C, OPTIMIZE_AFFINE_BRIGHTNESS> problem(
      reference_frame, target_frame, sensor_id, target_mask, trust_region_options_.sigma_huber_loss,
      trust_region_options_.affine_brightness_regularizer, t_t_r, affine_brightness_eps);

  auto result = energy::levenberg_marquardt_algorithm::solve(problem, options);

  covariance_t_t_r_ = problem.hessian()
                          .completeOrthogonalDecomposition()
                          .pseudoInverse()
                          .template topLeftCorner<Motion::DoF, Motion::DoF>();

  target_frame.T_w_agent_linearization_point = reference_frame.T_w_agent_linearization_point * t_t_r.inverse();
  target_frame.affine_brightness0 += affine_brightness_eps;
  target_frame.state_eps.setZero();
  return std::sqrt(result.energy / static_cast<Precision>(result.number_of_valid_residuals) / PatternSize);
}

template <energy::motion::Motion Motion, model::Model Model, int PatternSize, template <int> typename Grid2D, int C,
          bool OPTIMIZE_AFFINE_BRIGHTNESS>
Eigen::Matrix<Precision, Motion::DoF, Motion::DoF> EigenPoseAlignment<
    Motion, Model, PatternSize, Grid2D, C, OPTIMIZE_AFFINE_BRIGHTNESS>::tTargetReferenceCovariance() const {
  return covariance_t_t_r_;
}

template <energy::motion::Motion Motion, model::Model Model, int PatternSize, template <int> typename Grid2D, int C,
          bool OPTIMIZE_AFFINE_BRIGHTNESS>
EigenPoseAlignment<Motion, Model, PatternSize, Grid2D, C, OPTIMIZE_AFFINE_BRIGHTNESS>::~EigenPoseAlignment() = default;

#define PoseAlignmentEigenInstantiation(Motion, Model, PatternSize, Grid, C, OPTIMIZE_AFFINE_BRIGHTNESS)     \
  template class EigenPoseAlignment<energy::motion::Motion<Precision>, model::Model<Precision>, PatternSize, \
                                    features::Grid, C, OPTIMIZE_AFFINE_BRIGHTNESS>

PoseAlignmentEigenInstantiation(SE3, PinholeCamera, Pattern::kSize, CeresGrid, 1, true);
PoseAlignmentEigenInstantiation(SE3, PinholeCamera, Pattern::kSize, PixelMap, 1, true);
PoseAlignmentEigenInstantiation(SE3, PinholeCamera, 1, PixelMap, 1, true);
PoseAlignmentEigenInstantiation(SE3, PinholeCamera, 1, CeresGrid, 1, true);
PoseAlignmentEigenInstantiation(SE3, SimpleRadialCamera, 1, PixelMap, 1, true);

PoseAlignmentEigenInstantiation(SE3, PinholeCamera, 1, PixelMap, 32, true);
PoseAlignmentEigenInstantiation(SE3, PinholeCamera, 1, PixelMap, 128, true);

#undef PoseAlignmentEigenInstantiation

}  // namespace problem
}  // namespace energy
}  // namespace dsopp
