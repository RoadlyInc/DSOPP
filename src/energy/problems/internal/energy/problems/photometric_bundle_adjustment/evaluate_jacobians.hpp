#ifndef DSOPP_EVALUATE_JACOBIANS_HPP
#define DSOPP_EVALUATE_JACOBIANS_HPP

#include <deque>
#include <memory>

#include <tbb/parallel_for.h>

#include "common/patch/patch.hpp"
#include "energy/problems/photometric_bundle_adjustment/local_frame.hpp"
#include "energy/projector/camera_reproject.hpp"
#include "measures/similarity_measure_ssd.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"

#include "track/connections/frame_connection.hpp"

namespace dsopp {
namespace energy {
namespace problem {
template <typename Scalar, energy::motion::Motion Motion, typename Model, int PatternSize,
          template <int> typename Grid2D, int C, bool FIRST_ESTIMATE_JACOBIANS, bool OPTIMIZE_IDEPTHS,
          bool EVALUATE_JACOBIANS, bool NEW_EVALUATION_POINT, bool APPLY_HUBER_LOSS = false>
void evaluateJacobians(std::deque<std::unique_ptr<LocalFrame<Scalar, Motion, Model, PatternSize, Grid2D, C>>> &frames,
                       const Precision sigma_huber_loss = 0) {
  CHECK(APPLY_HUBER_LOSS or sigma_huber_loss == 0);
  const Scalar kSigmaHuberSqr = static_cast<Scalar>(sigma_huber_loss * sigma_huber_loss);
  using MotionCasted = typename Motion::template CastT<Scalar>;

  tbb::parallel_for(tbb::blocked_range<size_t>(0, frames.size()), [&](auto &frame_r) {
    for (size_t reference_idx = frame_r.begin(); reference_idx != frame_r.end(); ++reference_idx) {
      auto &reference_frame = frames[reference_idx];
      for (auto &target_frame : frames) {
        if (reference_frame->id == target_frame->id) {
          continue;
        }
        const Eigen::Matrix<Scalar, Motion::DoF, 1> t_log_w_r_eps =
            reference_frame->state_eps.template head<Motion::DoF>() +
            reference_frame->state_eps_step.template head<Motion::DoF>();
        const Eigen::Matrix<Scalar, Motion::DoF, 1> t_log_w_t_eps =
            target_frame->state_eps.template head<Motion::DoF>() +
            target_frame->state_eps_step.template head<Motion::DoF>();
        const Eigen::Matrix<Scalar, 2, 1> reference_affine_brightness_eps =
            reference_frame->state_eps.template tail<2>() + reference_frame->state_eps_step.template tail<2>();
        const Eigen::Matrix<Scalar, 2, 1> target_affine_brightness_eps =
            target_frame->state_eps.template tail<2>() + target_frame->state_eps_step.template tail<2>();

        const typename MotionCasted::Product t_t_r0 =
            target_frame->T_w_agent_linearization_point.inverse() * reference_frame->T_w_agent_linearization_point;
        const typename MotionCasted::Product t_t_r = t_t_r0.rightIncrement(t_log_w_r_eps).leftIncrement(-t_log_w_t_eps);
        const Eigen::Vector2<Scalar> reference_affine_brightness =

            reference_frame->affine_brightness0 + reference_affine_brightness_eps;
        const Eigen::Vector2<Scalar> target_affine_brightness =
            target_frame->affine_brightness0 + target_affine_brightness_eps;

        const Scalar brightness_change_scale = (target_frame->exposure_time / reference_frame->exposure_time) *
                                               std::exp(target_affine_brightness[0] - reference_affine_brightness[0]);

        const reprojection::ArrayReprojector<Scalar, Model, typename MotionCasted::Product> reprojector(
            reference_frame->model, target_frame->model, t_t_r);

        auto rightLogTransformer =
            (FIRST_ESTIMATE_JACOBIANS ? t_t_r0.rightLogTransformer() : t_t_r.rightLogTransformer())
                .template cast<Scalar>();
        auto leftLogTransformer = (FIRST_ESTIMATE_JACOBIANS ? t_t_r0.leftLogTransformer() : t_t_r.leftLogTransformer())
                                      .template cast<Scalar>();

        for (const auto &sensor_1 : reference_frame->sensors()) {
          for (const auto &sensor_2 : reference_frame->sensors()) {
            if (not reference_frame->residuals[{sensor_1, sensor_2}].contains(target_frame->id)) {
              continue;
            }
            const auto &target_mask = target_frame->masks.at(sensor_2);
            const auto &landmarks = reference_frame->active_landmarks[sensor_1];
            auto &residuals = reference_frame->residuals[{sensor_1, sensor_2}][target_frame->id];
            auto &grid = *target_frame->grids[sensor_2];
            CHECK_EQ(landmarks.size(), residuals.size());
            const size_t landmarks_size = landmarks.size();
            tbb::parallel_for(tbb::blocked_range<size_t>(0, landmarks_size), [&](auto r) {
              Scalar d_reference_affineBrightnessShift = 0;
              for (auto landmark_i = r.begin(); landmark_i != r.end(); ++landmark_i) {
                const auto &landmark = landmarks[landmark_i];
                if (landmark.is_marginalized && !landmark.to_marginalize) {
                  continue;
                }
                auto &residual = residuals[landmark_i];
                Eigen::Matrix<Scalar, PatternSize, C, PatchStorageOrder<C>> corrected_reference_intensities;
                residual.was_estimated = true;
                Eigen::Matrix<Scalar, 2, PatternSize> target_pattern;
                bool success = false;
                if constexpr (FIRST_ESTIMATE_JACOBIANS or not EVALUATE_JACOBIANS) {
                  success = reprojector.reprojectPattern(landmark.reference_pattern,
                                                         landmark.idepth + landmark.idepth_step, target_pattern);
                  success = success && (not FIRST_ESTIMATE_JACOBIANS or residual.reprojection_jacobians_valid);
                  d_reference_affineBrightnessShift = residual.brightness_change_scale;
                  corrected_reference_intensities = landmark.corrected_intensities;
                } else {
                  success = residual.reprojection_jacobians_valid =
                      reprojector.reprojectPattern(landmark.reference_pattern, landmark.idepth + landmark.idepth_step,
                                                   target_pattern, residual.d_u_idepth, residual.d_v_idepth,
                                                   residual.d_u_tReferenceTarget, residual.d_v_tReferenceTarget);

                  corrected_reference_intensities =
                      brightness_change_scale *
                      (landmark.patch - Eigen::Matrix<Scalar, PatternSize, C, PatchStorageOrder<C>>::Constant(
                                            reference_affine_brightness[1]));
                  d_reference_affineBrightnessShift = brightness_change_scale;
                }
                const bool kCheckBoundaries = false;
                success = success && target_mask.template valid<kCheckBoundaries>(target_pattern);
                if (!success) {
                  residual.connection_status_candidate = track::PointConnectionStatus::kOOB;
                }
                if (success and residual.connection_status == track::PointConnectionStatus::kOk) {
                  residual.connection_status_candidate = track::PointConnectionStatus::kOk;
                  Eigen::Matrix<Scalar, PatternSize, C, PatchStorageOrder<C>> target_patch;
                  Eigen::Vector<Scalar, PatternSize * C> d_intensity_u_diag;
                  Eigen::Vector<Scalar, PatternSize * C> d_intensity_v_diag;
                  if constexpr (EVALUATE_JACOBIANS) {
                    grid.Evaluate(target_pattern, target_patch, d_intensity_u_diag, d_intensity_v_diag);
                  } else {
                    grid.Evaluate(target_pattern, target_patch);
                  }
                  const Eigen::Matrix<Scalar, PatternSize, C, PatchStorageOrder<C>> residuals_left =
                      target_patch - Eigen::Matrix<Scalar, PatternSize, C, PatchStorageOrder<C>>::Constant(
                                         target_affine_brightness[1]);
                  const Eigen::Matrix<Scalar, PatternSize, C, PatchStorageOrder<C>> residuals_right =
                      brightness_change_scale *
                      (landmark.patch - Eigen::Matrix<Scalar, PatternSize, C, PatchStorageOrder<C>>::Constant(
                                            reference_affine_brightness[1]));

                  if constexpr (NEW_EVALUATION_POINT) {
                    Eigen::Map<Eigen::Matrix<Scalar, PatternSize, C, PatchStorageOrder<C>>> matrix_residuals(
                        residual.residuals.data());
                    measure::SimilarityMeasureSSD::residuals(residuals_left, residuals_right, matrix_residuals);
                    const Scalar residuals_squared_norm = residual.residuals.squaredNorm();
                    residual.energy = residuals_squared_norm * Scalar(0.5);
                    residual.huber_weight = 1;
                    if constexpr (APPLY_HUBER_LOSS) {
                      if (residuals_squared_norm > kSigmaHuberSqr) {
                        const Scalar residuals_norm = sqrt(residuals_squared_norm);
                        residual.huber_weight = static_cast<Scalar>(sigma_huber_loss) / residuals_norm;
                        residual.energy =
                            static_cast<Scalar>(sigma_huber_loss) * residuals_norm - kSigmaHuberSqr * Scalar(0.5);
                      }
                    }
                  }
                  if constexpr (EVALUATE_JACOBIANS) {
                    Eigen::Matrix<Scalar, PatternSize * C, Motion::Product::DoF> d_target_reference_state;
                    for (size_t i = 0; i < PatternSize; ++i) {
                      d_target_reference_state.template block<C, Motion::Product::DoF>(static_cast<int>(i * C), 0)
                          .noalias() = d_intensity_v_diag.template segment<C>(static_cast<int>(i * C)) *
                                       residual.d_v_tReferenceTarget.row(static_cast<int>(i));
                      d_target_reference_state.template block<C, Motion::Product::DoF>(static_cast<int>(i * C), 0)
                          .noalias() += d_intensity_u_diag.template segment<C>(static_cast<int>(i * C)) *
                                        residual.d_u_tReferenceTarget.row(static_cast<int>(i));
                    }

                    residual.d_target_state_eps.template leftCols<Motion::DoF>().noalias() =
                        -d_target_reference_state * leftLogTransformer;

                    residual.d_reference_state_eps.template leftCols<Motion::DoF>().noalias() =
                        d_target_reference_state * rightLogTransformer;

                    if constexpr (OPTIMIZE_IDEPTHS) {
                      for (size_t i = 0; i < PatternSize; ++i) {
                        residual.d_idepth.template segment<C>(static_cast<int>(i * C)).noalias() =
                            d_intensity_u_diag.template segment<C>(static_cast<int>(i * C)) *
                            residual.d_u_idepth[static_cast<int>(i)];
                        residual.d_idepth.template segment<C>(static_cast<int>(i * C)).noalias() +=
                            d_intensity_v_diag.template segment<C>(static_cast<int>(i * C)) *
                            residual.d_v_idepth[static_cast<int>(i)];
                      }
                    }

                    Eigen::Map<Eigen::Vector<Scalar, PatternSize * C>> corrected_reference_intensities_vector(
                        corrected_reference_intensities.data());
                    residual.d_reference_state_eps.col(Motion::DoF).noalias() = corrected_reference_intensities_vector;
                    residual.d_reference_state_eps.col(Motion::DoF + 1).setConstant(d_reference_affineBrightnessShift);

                    residual.d_target_state_eps.col(Motion::DoF).noalias() = -corrected_reference_intensities_vector;
                    residual.d_target_state_eps.col(Motion::DoF + 1).setConstant(-1);
                  }
                } else {
                  if constexpr (NEW_EVALUATION_POINT) {
                    residual.residuals.setZero();
                    residual.energy = 0;
                  }
                  if constexpr (EVALUATE_JACOBIANS) {
                    residual.d_reference_state_eps.setZero();
                    residual.d_target_state_eps.setZero();
                    residual.d_idepth.setZero();
                  }
                }
              }
            });
          }
        }
      }
    }
  });
}
}  // namespace problem
}  // namespace energy
}  // namespace dsopp

#endif  // DSOPP_EVALUATE_JACOBIANS_HPP
