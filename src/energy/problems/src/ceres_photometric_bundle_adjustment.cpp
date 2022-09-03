#include "energy/problems/photometric_bundle_adjustment/ceres_photometric_bundle_adjustment.hpp"

#include "energy/problems/photometric_bundle_adjustment/photometric_bundle_adjustment.hpp"

#include <ceres/types.h>
#include <type_traits>
#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/camera_model/pinhole/simple_radial.hpp"

#include "energy/problems/cost_functors/affine_brightness_regularization_cost_functor.hpp"
#include "energy/problems/depth_map.hpp"
#include "energy/problems/photometric_bundle_adjustment/bundle_adjustment_photometric_evaluation_callback.hpp"
#include "energy/problems/photometric_bundle_adjustment/ceres_photometric_bundle_adjustment_add_blocks.hpp"
#include "energy/problems/photometric_bundle_adjustment/covariance_matrices_of_relative_poses.hpp"
#include "energy/problems/photometric_bundle_adjustment/state_priors.hpp"
#include "features/camera/ceres_grid.hpp"
#include "features/camera/pixel_map.hpp"

#include "track/connections/frame_connection.hpp"
#include "track/frames/active_keyframe.hpp"
#include "track/landmarks/active_tracking_landmark.hpp"
#include "track/landmarks/tracking_landmark.hpp"
#define DEBUG_PHOTOMETRIC_BA COMPILE_HIDDEN_CODE

#if DEBUG_PHOTOMETRIC_BA

#include "common/image_tools/conversion.hpp"

#endif

namespace dsopp {
namespace energy {
namespace problem {
namespace {
template <energy::motion::Motion Motion, model::Model Model, int PatternSize, template <int> typename Grid2D, int C>
void updateResidual(LocalFrame<double, Motion, Model, PatternSize, Grid2D, C> &reference_frame,
                    LocalFrame<double, Motion, Model, PatternSize, Grid2D, C> &target_frame, size_t sensor_id) {
  if (target_frame.is_marginalized) {
    return;
  }
  auto &landmarks_frame = reference_frame.active_landmarks;
  for (auto &[sensor, landmarks] : landmarks_frame) {
    auto &residuals = reference_frame.residuals[std::make_pair(sensor, sensor)].at(target_frame.id);
    for (size_t landmark_index = 0; landmark_index < residuals.size(); landmark_index++) {
      auto &landmark = landmarks[landmark_index];
      cost_functors::BundleAdjustmentPhotometricCostFunctor<Motion, Grid2D<C>, Model, PatternSize, C> cost_functors(
          reference_frame.T_w_agent_linearization_point, target_frame.T_w_agent_linearization_point,
          reference_frame.exposure_time, reference_frame.affine_brightness0, target_frame.exposure_time,
          target_frame.affine_brightness0, *target_frame.grids.at(sensor_id), landmark.reference_pattern,
          landmark.patch, target_frame.masks.at(sensor_id), reference_frame.model.image_size(),
          target_frame.model.image_size());
      residuals[landmark_index].connection_status = cost_functors(
          reference_frame.state_eps.template head<Motion::DoF>(), target_frame.state_eps.template head<Motion::DoF>(),
          landmark.idepth, reference_frame.state_eps.template tail<2>(), target_frame.state_eps.template tail<2>(),
          reference_frame.intrinsic_parameters, target_frame.intrinsic_parameters, residuals[landmark_index].residuals);
    }
  }
}

template <energy::motion::Motion Motion, model::Model Model, int PatternSize, template <int> typename Grid2D, int C>
void updateResiduals(std::deque<std::unique_ptr<LocalFrame<double, Motion, Model, PatternSize, Grid2D, C>>> &frames,
                     size_t sensor_id) {
  for (size_t target_idx = 0; target_idx < frames.size(); ++target_idx) {
    auto &target_frame = *frames[target_idx];
    for (size_t reference_idx = 0; reference_idx < target_idx; reference_idx++) {
      auto &reference_frame = *frames[reference_idx];
      updateResidual<Motion, Model, PatternSize, Grid2D, C>(reference_frame, target_frame, sensor_id);
      updateResidual<Motion, Model, PatternSize, Grid2D, C>(target_frame, reference_frame, sensor_id);
    }
  }
}

template <energy::motion::Motion Motion, energy::model::Model Model, int PatternSize, template <int> typename Grid2D,
          int C>
Eigen::MatrixXd covarianceMatrixPosePose(
    std::deque<std::unique_ptr<LocalFrame<double, Motion, Model, PatternSize, Grid2D, C>>> &frames,
    ceres::Problem &problem, const int number_of_threads, const size_t sensor_id) {
  const int kBlockSize = Motion::DoF + 2;
  const size_t num_frame = frames.size();

  Eigen::MatrixXd covariance_matrix_pose_pose(kBlockSize * num_frame, kBlockSize * num_frame);
  covariance_matrix_pose_pose.setZero();

  ceres::Covariance::Options options_covariance;
  options_covariance.algorithm_type = ceres::DENSE_SVD;
  options_covariance.num_threads = number_of_threads;
  options_covariance.null_space_rank = -1;
  options_covariance.apply_loss_function = false;
  options_covariance.min_reciprocal_condition_number = 1e-14;
  ceres::Covariance covariance(options_covariance);

  std::vector<std::pair<const double *, const double *>> covariance_blocks;
  for (const auto &reference_frame : frames) {
    for (const auto &target_frame : frames) {
      covariance_blocks.push_back({reference_frame->state_eps.data(), target_frame->state_eps.data()});
    }
    for (auto &active_landmark : reference_frame->active_landmarks[sensor_id]) {
      covariance_blocks.push_back({&active_landmark.idepth, &active_landmark.idepth});
    }
  }
  covariance.Compute(covariance_blocks, &problem);
  for (size_t reference_idx = 0; reference_idx < frames.size(); reference_idx++) {
    for (size_t target_idx = 0; target_idx < frames.size(); target_idx++) {
      double covariance_vector[size_t(kBlockSize * kBlockSize)];
      covariance.GetCovarianceBlock(frames[reference_idx]->state_eps.data(), frames[target_idx]->state_eps.data(),
                                    covariance_vector);
      covariance_matrix_pose_pose.template block<kBlockSize, kBlockSize>(int(reference_idx) * kBlockSize,
                                                                         int(target_idx) * kBlockSize) =
          Eigen::Map<const Eigen::Matrix<double, kBlockSize, kBlockSize, Eigen::RowMajor>>(covariance_vector);
    }
  }

  for (size_t reference_idx = 0; reference_idx < frames.size(); reference_idx++) {
    auto &frame = *frames[reference_idx];
    for (auto &active_landmark : frame.active_landmarks[sensor_id]) {
      double idepth_variance[1] = {0};
      covariance.GetCovarianceBlock(&active_landmark.idepth, &active_landmark.idepth, idepth_variance);
      active_landmark.inv_hessian_idepth_idepth = idepth_variance[0];
    }
  }

  return covariance_matrix_pose_pose;
}
}  // namespace

template <energy::motion::Motion Motion, model::Model Model, int PatternSize, template <int> typename Grid2D,
          bool OPTIMIZE_POSES, bool OPTIMIZE_IDEPTHS, bool FIRST_ESTIMATE_JACOBIANS, bool OPTIMIZE_CAMERA, int C>
CeresPhotometricBundleAdjustment<Motion, Model, PatternSize, Grid2D, OPTIMIZE_POSES, OPTIMIZE_IDEPTHS,
                                 FIRST_ESTIMATE_JACOBIANS, OPTIMIZE_CAMERA, C>::
    CeresPhotometricBundleAdjustment(const TrustRegionPhotometricBundleAdjustmentOptions<double> &trust_region_options,
                                     bool estimate_uncertainty, bool use_analytic_diff, size_t level)
    : PhotometricBundleAdjustment<double, Motion, Model, PatternSize, Grid2D, OPTIMIZE_POSES, OPTIMIZE_IDEPTHS,
                                  FIRST_ESTIMATE_JACOBIANS, C>(estimate_uncertainty),
      use_analytic_diff_(use_analytic_diff),
      level_(level),
      trust_region_options_(trust_region_options) {
  evaluation_callback_ = std::make_unique<BundleAdjustmentPhotometricEvaluationCallback<
      double, Motion, Model, PatternSize, Grid2D, C, FIRST_ESTIMATE_JACOBIANS, OPTIMIZE_IDEPTHS>>(this->frames_);
}

template <energy::motion::Motion Motion, model::Model Model, int PatternSize, template <int> typename Grid2D,
          bool OPTIMIZE_POSES, bool OPTIMIZE_IDEPTHS, bool FIRST_ESTIMATE_JACOBIANS, bool OPTIMIZE_CAMERA, int C>
Precision
CeresPhotometricBundleAdjustment<Motion, Model, PatternSize, Grid2D, OPTIMIZE_POSES, OPTIMIZE_IDEPTHS,
                                 FIRST_ESTIMATE_JACOBIANS, OPTIMIZE_CAMERA, C>::solve(const size_t number_of_threads) {
  auto loss_function = std::make_unique<ceres::HuberLoss>(trust_region_options_.sigma_huber_loss);
  auto *ordering = new ceres::ParameterBlockOrdering();
  size_t sensor_id = this->frames_[0]->sensors()[0];
  ceres::Problem::Options problem_options;
  problem_options.evaluation_callback = this->evaluation_callback_.get();
  problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  problem_options.disable_all_safety_checks = false;
#ifdef NDEBUG
  problem_options.disable_all_safety_checks = true;
#endif
  ceres::Problem problem(problem_options);

  addFramesToProblem<Motion, Model, PatternSize, Grid2D, OPTIMIZE_POSES, OPTIMIZE_IDEPTHS, OPTIMIZE_CAMERA, C>(
      this->frames_, sensor_id, problem, loss_function.get(), ordering,
      trust_region_options_.affine_brightness_regularizer, use_analytic_diff_);

  ceres::Solver::Options options;
  options.num_threads = static_cast<int>(number_of_threads);
  options.max_num_iterations = static_cast<int>(trust_region_options_.max_iterations);
  options.initial_trust_region_radius = trust_region_options_.initial_trust_region_radius;
  options.function_tolerance = trust_region_options_.function_tolerance;
  options.gradient_tolerance = 0;
  options.parameter_tolerance = trust_region_options_.parameter_tolerance;
  options.logging_type = ceres::SILENT;
  options.linear_solver_ordering.reset(ordering);

  if (OPTIMIZE_IDEPTHS) {
    options.linear_solver_type = ceres::DENSE_SCHUR;
  } else {
    options.linear_solver_type = ceres::DENSE_QR;
  }

  ceres::Solver::Summary summary;
  if (use_analytic_diff_ && FIRST_ESTIMATE_JACOBIANS) {
    this->firstEstimateJacobians();
  }

  Solve(options, &problem, &summary);
  size_t minimum_valid_reprojections_num = 1;

  if (!use_analytic_diff_) {
    updateResiduals<Motion, Model, PatternSize, Grid2D, C>(this->frames_, sensor_id);
  }

  this->relinearizeSystem();

  if (this->estimate_uncertainty_) {
    this->firstEstimateJacobians();
    covarianceMatricesOfRelativePoses(
        this->frames_,
        covarianceMatrixPosePose(this->frames_, problem, static_cast<int>(number_of_threads), sensor_id));
  }
  this->updatePointStatuses(minimum_valid_reprojections_num, trust_region_options_.sigma_huber_loss);

#if DEBUG_PHOTOMETRIC_BA
  constexpr static uint8_t kColors[][3] = {{230, 25, 75},  {60, 180, 75},   {255, 225, 25}, {0, 130, 200},
                                           {245, 130, 48}, {145, 30, 180},  {70, 240, 240}, {240, 50, 230},
                                           {210, 245, 60}, {250, 190, 190}, {0, 128, 128},  {230, 190, 255},
                                           {170, 110, 40}, {255, 250, 200}, {128, 0, 0},    {170, 255, 195},
                                           {128, 128, 0},  {255, 215, 180}, {0, 0, 128},    {128, 128, 128}};
  if constexpr (std::is_same<Grid2D<1>, features::PixelMap<1>>::value) {
    std::vector<LocalFrame<double, Motion, Model, PatternSize, Grid2D, C> *> debug_frames;
    for (const auto &frame : this->frames_) {
      if (not frame->is_marginalized) {
        debug_frames.push_back(frame.get());
      }
    }
    if (level_ != 0 or debug_frames.size() < 3) {
      return static_cast<Precision>(summary.final_cost);
    }
    const int kColNum = 4;
    const int sensor = 0;
    int cols = std::min(static_cast<int>(debug_frames.size()), kColNum);
    int rows = (static_cast<int>(debug_frames.size()) + kColNum - 1) / kColNum;
    int width = static_cast<int>(debug_frames[0]->grids.at(0)->width());
    int height = static_cast<int>(debug_frames[0]->grids.at(0)->height());
    cv::Mat full(height * rows, width * cols, CV_8UC3, cv::Scalar(0));
    std::vector<cv::Mat> images(debug_frames.size());

    for (size_t i = 0; i < images.size(); i++) {
      const auto &frame = *debug_frames[i];
      cv::cvtColor(common::image_tools::pixelMap2Mat1C(*frame.grids.at(0)), images[i], cv::COLOR_GRAY2BGR);
    }
    for (size_t i = 0; i < images.size(); i++) {
      const auto &color = kColors[i];
      const auto &frame_reference = *debug_frames[i];
      for (const auto &landmark : frame_reference.active_landmarks.at(sensor)) {
        cv::circle(images[i],
                   cv::Point(static_cast<int>(landmark.projection[0]), static_cast<int>(landmark.projection[1])), 5,
                   cv::Scalar(color[2], color[1], color[0]), -1);
      }
    }
    for (size_t i = 0; i < images.size(); i++) {
      for (size_t j = 0; j < images.size(); j++) {
        if (i == j) continue;
        const auto &color = kColors[i];
        const auto &frame_reference = *debug_frames[i];
        const auto &frame_target = *debug_frames[j];
        const auto reprojections = frame_reference.residuals.at(std::make_pair(sensor, sensor)).at(frame_target.id);

        reprojection::ArrayReprojector<double, Model, typename Motion::template CastT<double>::Product> reprojector(
            frame_reference.model, frame_target.model,
            frame_target.tWorldAgent().inverse() * frame_reference.tWorldAgent());

        for (size_t landmark_id = 0; landmark_id < frame_reference.active_landmarks.at(sensor).size(); landmark_id++) {
          Eigen::Vector2d reprojection;
          const auto &landmark = frame_reference.active_landmarks.at(sensor).at(landmark_id);
          if (reprojector.reproject(landmark.projection, landmark.idepth, reprojection)) {
            if (reprojections[landmark_id].connection_status == track::PointConnectionStatus::kOk) {
              cv::circle(images[j], cv::Point(static_cast<int>(reprojection[0]), static_cast<int>(reprojection[1])), 5,
                         cv::Scalar(color[2], color[1], color[0]), -1);
            } else {
              cv::circle(images[j], cv::Point(static_cast<int>(reprojection[0]), static_cast<int>(reprojection[1])), 5,
                         cv::Scalar(color[2], color[1], color[0]));
            }
          }
        }
      }
    }
    for (int i = 0; i < static_cast<int>(images.size()); i++) {
      int row = i / kColNum;
      int col = i % kColNum;
      images[static_cast<size_t>(i)].copyTo(full(cv::Rect(col * width, row * height, width, height)));
    }
    cv::resize(full, full, cv::Size(), 0.7, 0.7);
    cv::imshow("DEBUG_PHOTOMETRIC_BA", full);
    cv::waitKey();
  }
#endif

  return static_cast<Precision>(summary.final_cost);
}
template <energy::motion::Motion Motion, model::Model Model, int PatternSize, template <int> typename Grid2D,
          bool OPTIMIZE_POSES, bool OPTIMIZE_IDEPTHS, bool FIRST_ESTIMATE_JACOBIANS, bool OPTIMIZE_CAMERA, int C>
void CeresPhotometricBundleAdjustment<Motion, Model, PatternSize, Grid2D, OPTIMIZE_POSES, OPTIMIZE_IDEPTHS,
                                      FIRST_ESTIMATE_JACOBIANS, OPTIMIZE_CAMERA,
                                      C>::updateLocalFrame(const track::ActiveKeyframe<Motion> &frame) {
  auto local_frame = this->getLocalFrame(frame.timestamp());
  CHECK(local_frame) << "Cannot update frame, there is no local copy in the solver";
  bool recently_marginalized = frame.isMarginalized() && !local_frame->is_marginalized;
  local_frame->is_marginalized = frame.isMarginalized();
  local_frame->update(frame, frame.connections());
  size_t sensor_id = frame.sensors()[0];
  if (!recently_marginalized) return;
  // remove all marginalized frames, that have no connections with active frames
  for (auto frame_iter = this->frames_.begin(); frame_iter != this->frames_.end();) {
    auto &residuals = (*frame_iter)->residuals.at({sensor_id, sensor_id});
    if (!(*frame_iter)->is_marginalized) {
      ++frame_iter;
      continue;
    }
    bool has_connections = false;
    for (auto &target_frame : this->frames_) {
      if (!target_frame->is_marginalized && residuals.contains(target_frame->id)) {
        has_connections = true;
      }
    }
    if (!has_connections) {
      frame_iter = this->frames_.erase(frame_iter);
    } else
      ++frame_iter;
  }
}

template <energy::motion::Motion Motion, model::Model Model, int PatternSize, template <int> typename Grid2D,
          bool OPTIMIZE_POSES, bool OPTIMIZE_IDEPTHS, bool FIRST_ESTIMATE_JACOBIANS, bool OPTIMIZE_CAMERA, int C>
Model CeresPhotometricBundleAdjustment<Motion, Model, PatternSize, Grid2D, OPTIMIZE_POSES, OPTIMIZE_IDEPTHS,
                                       FIRST_ESTIMATE_JACOBIANS, OPTIMIZE_CAMERA,
                                       C>::optimizedCameraModel(size_t frame_id) const {
  return Model(this->frames_.at(frame_id)->model.image_size(),
               this->frames_.at(frame_id)->intrinsic_parameters.template cast<Precision>());
}

template <energy::motion::Motion Motion, model::Model Model, int PatternSize, template <int> typename Grid2D,
          bool OPTIMIZE_POSES, bool OPTIMIZE_IDEPTHS, bool FIRST_ESTIMATE_JACOBIANS, bool OPTIMIZE_CAMERA, int C>
CeresPhotometricBundleAdjustment<Motion, Model, PatternSize, Grid2D, OPTIMIZE_POSES, OPTIMIZE_IDEPTHS,
                                 FIRST_ESTIMATE_JACOBIANS, OPTIMIZE_CAMERA, C>::~CeresPhotometricBundleAdjustment() =
    default;

/// \cond DO_NOT_DOCUMENT

#define PBACeresInstantiation(Motion, Model, PatchSize, Grid, OPTIMIZE_POSES, OPTIMIZE_IDEPTHS,                \
                              FIRST_ESTIMATE_JACOBIANS, OPTIMIZE_CAMERA, C)                                    \
  template class CeresPhotometricBundleAdjustment<energy::motion::Motion<Precision>, model::Model<Precision>,  \
                                                  PatchSize, features::Grid, OPTIMIZE_POSES, OPTIMIZE_IDEPTHS, \
                                                  FIRST_ESTIMATE_JACOBIANS, OPTIMIZE_CAMERA, C>

PBACeresInstantiation(SE3, PinholeCamera, Pattern::kSize, PixelMap, true, true, true, false, 1);
PBACeresInstantiation(SE3, PinholeCamera, Pattern::kSize, PixelMap, true, true, false, true, 1);
PBACeresInstantiation(SE3, PinholeCamera, Pattern::kSize, PixelMap, true, false, false, false, 1);
PBACeresInstantiation(SE3, PinholeCamera, Pattern::kSize, PixelMap, false, false, false, true, 1);
PBACeresInstantiation(SE3, PinholeCamera, Pattern::kSize, CeresGrid, true, false, false, false, 1);
PBACeresInstantiation(SE3, PinholeCamera, 1, PixelMap, true, false, false, false, 1);
PBACeresInstantiation(SE3, SimpleRadialCamera, Pattern::kSize, PixelMap, true, true, false, true, 1);
PBACeresInstantiation(SE3, SimpleRadialCamera, Pattern::kSize, PixelMap, true, true, true, false, 1);

#undef PBACeresInstantiation

/// \endcond
}  // namespace problem
}  // namespace energy
}  // namespace dsopp
