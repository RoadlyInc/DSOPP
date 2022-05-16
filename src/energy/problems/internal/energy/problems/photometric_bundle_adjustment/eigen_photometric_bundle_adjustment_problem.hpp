#ifndef DSOPP_ENERGY_PROBLEMS_PHOTOMETRIC_BUNDLE_ADJUSTMENT_PROBLEM_HPP_
#define DSOPP_ENERGY_PROBLEMS_PHOTOMETRIC_BUNDLE_ADJUSTMENT_PROBLEM_HPP_

#include <mutex>

#include <tbb/parallel_for.h>
#include <tbb/task_group.h>

#include "common/settings.hpp"
#include "energy/motion/motion.hpp"
#include "energy/normal_linear_system.hpp"
#include "energy/problems/photometric_bundle_adjustment/hessian_block_evaluation.hpp"
#include "energy/problems/photometric_bundle_adjustment/local_frame.hpp"
#include "energy/problems/photometric_bundle_adjustment/state_priors.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"

namespace dsopp::energy::problem {
Eigen::MatrixX<Precision> pseudoInverse(const Eigen::MatrixX<Precision> &origin, const int number_of_nullspaces);

void changeResidualStatuses(auto &frames, bool accept = true) {
  for (auto &f : frames) {
    for (auto &[_, m] : f->residuals) {
      for (auto &[__, v] : m) {
        for (auto &residual : v) {
          if (accept) {
            residual.connection_status = residual.connection_status_candidate;
          } else {
            // reject
            residual.connection_status_candidate = residual.connection_status;
          }
        }
      }
    }
  }
}

template <energy::motion::Motion Motion, class CameraModel, int PatternSize, template <int> typename Grid2D, int C,
          typename EigenDerivedC>
void evaluateLinearSystemPrior(
    std::deque<std::unique_ptr<LocalFrame<Precision, Motion, CameraModel, PatternSize, Grid2D, C>>> &frames,
    NormalLinearSystem<> &system_prior, const Eigen::MatrixBase<EigenDerivedC> &affineBrightnessRegularizer,
    const Precision fixedStateRegularizer, bool for_marginalized = false) {
  const int kBlockSize = Motion::DoF + 2;

  for (size_t frame_index = 0; frame_index < frames.size(); ++frame_index) {
    auto &frame = frames[frame_index];
    if (frame->to_marginalize != for_marginalized) continue;

    const int frame_block = kBlockSize * static_cast<int>(frame_index);

    if (frame->frame_parameterization == FrameParameterization::kFixed) {
      NormalLinearSystem<Precision, kBlockSize> fixed_frame_prior;
      fixed_frame_prior.H = Eigen::Vector<Precision, kBlockSize>::Constant(fixedStateRegularizer).asDiagonal();
      fixed_frame_prior.b = fixedStateRegularizer * frame->state_eps;
      system_prior.addToBlock(frame_block, fixed_frame_prior);
    } else {
      auto affine_brightness = frame->affine_brightness0 + frame->state_eps.template tail<2>();

      auto affine_brightness_prior =
          hessian_prior::AffineBrightnessPrior::priorSystem(affine_brightness, affineBrightnessRegularizer);
      system_prior.addToBlock(frame_block + Motion::DoF, affine_brightness_prior);
    }

    if (frame_index != frames.size() - 1) {
      const int next_frame = kBlockSize * static_cast<int>(frame_index + 1);

      time::duration frame_duration = frames[frame_index + 1]->timestamp - frame->timestamp;

      auto [motion_prior_reference, motion_prior_target] = hessian_prior::MotionPrior<Motion>::systemPrior(
          frame->tWorldAgent().inverse() * frames[frame_index + 1]->tWorldAgent(),
          frame->T_w_agent_linearization_point.inverse() * frames[frame_index + 1]->T_w_agent_linearization_point,
          &frame->model, frame_duration);
      system_prior.addToBlock(frame_block, motion_prior_reference);
      system_prior.addToBlock(next_frame, motion_prior_target);
    }
  }
}

template <energy::motion::Motion Motion, typename Model, int PatternSize, template <int> typename Grid2D, int C>
Eigen::VectorX<Precision> stateEpsStacked(
    std::deque<std::unique_ptr<LocalFrame<Precision, Motion, Model, PatternSize, Grid2D, C>>> &frames,
    bool with_step = false) {
  const int kBlockSize = Motion::DoF + 2;
  Eigen::VectorX<Precision> state(static_cast<int>(frames.size()) * kBlockSize);
  tbb::parallel_for(tbb::blocked_range<size_t>(0, frames.size()), [&](auto r) {
    for (auto i = r.begin(); i != r.end(); ++i) {
      state.segment<kBlockSize>(static_cast<int>(i) * kBlockSize) = frames[i]->state_eps;
      if (with_step) state.segment<kBlockSize>(static_cast<int>(i) * kBlockSize) += frames[i]->state_eps_step;
    }
  });
  return state;
}
template <bool FOR_MARGINALIZED = false, energy::motion::Motion Motion, class Model, int PatternSize,
          template <int> typename Grid2D, int C>
std::pair<Precision, int> calculateLandmarksEnergy(
    const std::deque<std::unique_ptr<LocalFrame<Precision, Motion, Model, PatternSize, Grid2D, C>>> &frames,
    size_t sensor_id) {
  Precision energy = 0;
  int number_of_valid_residuals = 0;
  std::mutex mutex;
  tbb::parallel_for(tbb::blocked_range<size_t>(0, frames.size()), [&](auto r) {
    Precision local_energy = 0;
    int local_number_of_valid_residuals = 0;

    for (auto reference_frame_index = r.begin(); reference_frame_index != r.end(); ++reference_frame_index) {
      auto &reference_frame = *frames[reference_frame_index];
      auto &frame_residuals = reference_frame.residuals[{sensor_id, sensor_id}];
      for (size_t target_frame_index = 0; target_frame_index < frames.size(); ++target_frame_index) {
        auto &target_frame = *frames[target_frame_index];
        if (reference_frame.id == target_frame.id) {
          continue;
        }
        if (!frame_residuals.contains(target_frame.id)) {
          continue;
        }

        const auto &residuals = frame_residuals.at(target_frame.id);
        const auto &landmarks = reference_frame.active_landmarks.at(sensor_id);
        auto landmark_iter = landmarks.begin();
        const auto landmark_end = landmark_iter + static_cast<int>(residuals.size());
        auto residuals_iter = residuals.begin();
        for (; landmark_iter != landmark_end; ++landmark_iter, ++residuals_iter) {
          const auto &landmark = *landmark_iter;
          if constexpr (FOR_MARGINALIZED) {
            if (!landmark.to_marginalize) continue;
          } else {
            if (landmark.is_marginalized) continue;
          }
          const auto &residual = *residuals_iter;
          local_energy += residual.energy;
          if (residual.energy > 0) {
            local_number_of_valid_residuals++;
          }
        }
      }
    }
    {
      std::lock_guard guard(mutex);
      energy += local_energy;
      number_of_valid_residuals += local_number_of_valid_residuals;
    }
  });
  return {energy, number_of_valid_residuals};
}

template <energy::motion::Motion Motion, class CameraModel, int PatternSize, template <int> typename Grid2D, int C>
void updateMarginalizedLinearSystem(
    std::deque<std::unique_ptr<LocalFrame<Precision, Motion, CameraModel, PatternSize, Grid2D, C>>> &frames,
    size_t sensor_id, NormalLinearSystem<double> &system_marginalized, Precision &energy_marginalized,
    const Eigen::Vector2<Precision> &affineBrightnessRegularizer, const Precision fixedStateRegularizer) {
  const int kBlockSize = Motion::DoF + 2;
  const int num_frame = static_cast<int>(frames.size());

  NormalLinearSystem<> system_marginalized_points_schur(kBlockSize * num_frame);
  system_marginalized_points_schur.setZero();
  tbb::task_group depth_group;
  depth_group.run([&]() {
    evaluateLinearSystemPoseDepthSchurComplement<true>(frames, sensor_id, system_marginalized_points_schur);
  });

  NormalLinearSystem<> system_marginalized_points_pose(kBlockSize * num_frame);
  system_marginalized_points_pose.setZero();
  evaluateLinearSystemPosePose<true>(frames, sensor_id, system_marginalized_points_pose);
  depth_group.wait();

  NormalLinearSystem<> system_marginalized_points = system_marginalized_points_pose - system_marginalized_points_schur;
  auto state = stateEpsStacked(frames);
  // for details see eq 8.15 in the DSO paper
  energy_marginalized += calculateLandmarksEnergy<true>(frames, sensor_id).first +
                         state.dot(system_marginalized_points.H * state) - state.dot(system_marginalized_points.b);
  system_marginalized_points.b -= system_marginalized_points.H * state;

  system_marginalized += system_marginalized_points.cast<double>();

  for (auto &frame : frames) {
    for (auto &landmark : frame->active_landmarks[sensor_id]) landmark.to_marginalize = false;
  }

  // move marginalized frames to the marginalized part;
  std::vector<int> marginalized_part;
  for (size_t i = 0; i < frames.size(); i++) {
    for (int parameter = 0; parameter < kBlockSize; parameter++) {
      int idx = static_cast<int>(i) * kBlockSize + parameter;
      if (frames[i]->to_marginalize) {
        marginalized_part.push_back(idx);
      }
    }
  }
  if (marginalized_part.empty()) {
    return;
  }

  NormalLinearSystem<> system_prior(kBlockSize * num_frame);
  system_prior.setZero();
  evaluateLinearSystemPrior(frames, system_prior, affineBrightnessRegularizer, fixedStateRegularizer, true);
  system_prior.b -= system_prior.H * state;
  system_marginalized += system_prior.cast<double>();

  system_marginalized.reduce_system(marginalized_part);

  frames.erase(std::remove_if(frames.begin(), frames.end(), [](auto &frame) { return frame->to_marginalize; }),
               frames.end());
}
template <energy::motion::Motion Motion, model::Model Model, int PatternSize, template <int> typename Grid2D,
          bool OPTIMIZE_IDEPTHS, bool FIRST_ESTIMATE_JACOBIANS, int C>
Eigen::MatrixX<Precision> covarianceMatrixPosePose(
    std::deque<std::unique_ptr<LocalFrame<Precision, Motion, Model, PatternSize, Grid2D, C>>> &frames,
    const size_t sensor_id, const NormalLinearSystem<double> &system_marginalized,
    const Eigen::Vector2<Precision> &affine_brightness_regularizer, const Precision fixedStateRegularizer) {
  const int kBlockSize = Motion::DoF + 2;
  const int num_frame = static_cast<int>(frames.size());

  Eigen::MatrixX<Precision> covariance_matrix_pose_pose(kBlockSize * num_frame, kBlockSize * num_frame);
  covariance_matrix_pose_pose.setZero();

  evaluateJacobians<Precision, Motion, Model, PatternSize, Grid2D, C, FIRST_ESTIMATE_JACOBIANS, OPTIMIZE_IDEPTHS, true,
                    true, false>(frames);

  NormalLinearSystem<> system_pose(kBlockSize * num_frame);
  NormalLinearSystem<> system_schur(kBlockSize * num_frame);
  NormalLinearSystem<> system_full(kBlockSize * num_frame);

  system_pose.setZero();
  system_schur.setZero();

  tbb::task_group poses_group;
  poses_group.run([&]() {
    evaluateLinearSystemPosePose(frames, sensor_id, system_pose);
    evaluateLinearSystemPrior(frames, system_pose, affine_brightness_regularizer, fixedStateRegularizer);
  });
  if constexpr (OPTIMIZE_IDEPTHS) {
    evaluateLinearSystemPoseDepthSchurComplement(frames, sensor_id, system_schur);
  }
  poses_group.wait();

  system_full = system_pose - system_schur + system_marginalized.cast<Precision>();

  int scale_nullspace = OPTIMIZE_IDEPTHS;
  covariance_matrix_pose_pose = pseudoInverse(system_full.H, scale_nullspace);

  return covariance_matrix_pose_pose;
}

/**
 * \brief Class representing eigen photometric bundle adjustment problem
 *
 * @tparam Motion motion type
 * @tparam calibration camera calibration type
 * @tparam PatternSize pattern size
 * @tparam Grid2D image grid type, stores intensities, gradients
 * @tparam OPTIMIZE_IDEPTHS ``true`` to optimize inverse depths
 * @tparam FIRST_ESTIMATE_JACOBIANS ``true`` to use first estimate jacobian
 * @tparam C channels in each individual image
 */
template <energy::motion::Motion Motion, model::Model Model, int PatternSize, template <int> typename Grid2D,
          bool OPTIMIZE_POSES, bool OPTIMIZE_IDEPTHS, bool FIRST_ESTIMATE_JACOBIANS, int C>
class PhotometricBundleAdjustmentProblem {
  /** optimizable block size for each frame */
  static const int kBlockSize = Motion::DoF + 2;

 public:
  /**
   * @param frames local frames
   * @param sensor_id sensor id
   * @param sigma_huber_loss huber loss threshold
   * @param system_marginalized accumulated marginalized system
   * @param energy_marginalized energy of marignalized system
   * @param affine_brightness_regularizer regularizer for affine brightness
   * @param fixed_pose_regularizer regularizer for fixing pose
   */
  PhotometricBundleAdjustmentProblem(
      std::deque<std::unique_ptr<LocalFrame<Precision, Motion, Model, PatternSize, Grid2D, C>>> &frames,
      const size_t sensor_id, const Precision sigma_huber_loss, const NormalLinearSystem<> &system_marginalized,
      Precision energy_marginalized, const Eigen::Vector2<Precision> &affine_brightness_regularizer,
      const Precision fixed_pose_regularizer)
      : frames_(frames),
        sensor_id_(sensor_id),
        sigma_huber_loss_(sigma_huber_loss),
        system_marginalized_(system_marginalized),
        energy_marginalized_(energy_marginalized),
        affine_brightness_regularizer_(affine_brightness_regularizer),
        fixed_pose_regularizer_(fixed_pose_regularizer),
        system_pose_(kBlockSize * static_cast<int>(frames_.size())),
        system_schur_(kBlockSize * static_cast<int>(frames_.size())) {}

  /**
   * calculates energy in current state
   * @return energy, number of active residuals
   */
  std::pair<Precision, int> calculateEnergy() {
    evaluateJacobians<Precision, Motion, Model, PatternSize, Grid2D, C, FIRST_ESTIMATE_JACOBIANS, OPTIMIZE_IDEPTHS,
                      false, true, true>(frames_, sigma_huber_loss_);
    Precision energy = energy_marginalized_;
    int number_of_valid_residuals = 0;
    auto state = stateEpsStacked(frames_, true);

    energy += system_marginalized_.b.dot(state) +
              state.dot(system_marginalized_.H * state) / 2;  // see eq 8.19 in the DSO paper
    for (size_t reference_frame_index = 0; reference_frame_index < frames_.size(); ++reference_frame_index) {
      auto &reference_frame = *frames_[reference_frame_index];
      auto affine_brightness = reference_frame.affine_brightness0 + reference_frame.state_eps.template tail<2>() +
                               reference_frame.state_eps_step.template tail<2>();

      energy += hessian_prior::AffineBrightnessPrior::energyTerm(affine_brightness, affine_brightness_regularizer_);
      if (reference_frame_index != frames_.size() - 1) {
        time::duration frame_duration = frames_[reference_frame_index + 1]->timestamp - reference_frame.timestamp;

        energy += hessian_prior::MotionPrior<Motion>::energyTerm(
            reference_frame.tWorldAgent().inverse() * frames_[reference_frame_index + 1]->tWorldAgent(),
            &reference_frame.model, frame_duration);
      }
    }
    Precision landmarks_energy = 0;
    std::tie(landmarks_energy, number_of_valid_residuals) = calculateLandmarksEnergy(frames_, sensor_id_);

    return {energy + landmarks_energy, number_of_valid_residuals};
  }

  /**
   * linearize system
   */
  void linearize() {
    evaluateJacobians<Precision, Motion, Model, PatternSize, Grid2D, C, FIRST_ESTIMATE_JACOBIANS, OPTIMIZE_IDEPTHS,
                      true, true, true>(frames_, sigma_huber_loss_);
    system_pose_.setZero();
    system_schur_.setZero();
    tbb::task_group poses_group;
    poses_group.run([&]() {
      evaluateLinearSystemPosePose(frames_, sensor_id_, system_pose_);
      evaluateLinearSystemPrior(frames_, system_pose_, affine_brightness_regularizer_, fixed_pose_regularizer_);
    });
    if constexpr (OPTIMIZE_IDEPTHS) {
      evaluateLinearSystemPoseDepthSchurComplement(frames_, sensor_id_, system_schur_);
    }
    poses_group.wait();
  }

  /**
   * estiamte current step
   * @param levenberg_marquardt_regularizer lm regularizer
   */
  void calculateStep(const Precision levenberg_marquardt_regularizer) {
    const int num_frame = static_cast<int>(frames_.size());
    Eigen::VectorX<Precision> step(kBlockSize * num_frame);
    step.setZero();

    auto state = stateEpsStacked(frames_);
    NormalLinearSystem<> system_full = system_pose_ + system_marginalized_;
    system_full.H.diagonal() += system_pose_.H.diagonal() * levenberg_marquardt_regularizer;
    system_full += system_schur_ * (-1.0_p / (1.0_p + levenberg_marquardt_regularizer));
    system_full.b += system_marginalized_.H * state;
    step = system_full.solve();
    for (size_t frame_id = 0; frame_id < frames_.size(); ++frame_id) {
      auto &frame = *frames_[frame_id];
      auto frame_step = step.template segment<kBlockSize>(kBlockSize * static_cast<int>(frame_id));
      frame.state_eps_step = -frame_step;
    }
    if constexpr (OPTIMIZE_IDEPTHS) {
      calculateIdepths(frames_, sensor_id_, step, levenberg_marquardt_regularizer);
    }
  }
  /**
   * accept current step
   * @return state, step squared norms
   */
  std::pair<Precision, Precision> acceptStep() {
    Precision state_squared_norm = 0;
    Precision step_squared_norm = 0;
    for (auto &frame : frames_) {
      state_squared_norm += frame->state_eps.squaredNorm();
      state_squared_norm += frame->affine_brightness0.squaredNorm();

      frame->state_eps += frame->state_eps_step;
      step_squared_norm += frame->state_eps_step.squaredNorm();

      frame->state_eps_step.setZero();
      if constexpr (OPTIMIZE_IDEPTHS) {
        for (auto &landmark : frame->active_landmarks[sensor_id_]) {
          state_squared_norm += landmark.idepth * landmark.idepth;
          landmark.idepth += landmark.idepth_step;
          step_squared_norm += landmark.idepth_step * landmark.idepth_step;
          landmark.idepth_step = 0;
        }
      }
    }
    changeResidualStatuses(frames_);
    return {state_squared_norm, step_squared_norm};
  }
  /**
   * reject current step
   */
  void rejectStep() {
    for (auto &frame : frames_) {
      frame->state_eps_step.setZero();
      if constexpr (OPTIMIZE_IDEPTHS) {
        for (auto &landmark : frame->active_landmarks[sensor_id_]) {
          landmark.idepth_step = 0;
        }
      }
    }
    changeResidualStatuses(frames_, false);
  }
  /**
   * stop status
   * @return ``true`` if no more steps could be done
   */
  bool stop() { return false; }

 private:
  /** optimizable frames */
  std::deque<std::unique_ptr<LocalFrame<Precision, Motion, Model, PatternSize, Grid2D, C>>> &frames_;
  /** sensor id */
  const size_t sensor_id_;
  /** huber loss threshold */
  const Precision sigma_huber_loss_;
  /** accumulated marginalized system */
  NormalLinearSystem<> system_marginalized_;
  /** energy of marginalized system */
  const Precision energy_marginalized_;
  /** regularizer for affine brightness */
  const Eigen::Vector2<Precision> &affine_brightness_regularizer_;
  /** regularizer for fixed pose */
  const Precision fixed_pose_regularizer_;

  /** Hessian for poses */
  NormalLinearSystem<> system_pose_;
  /** schur complement ``multiplier`` */
  NormalLinearSystem<> system_schur_;
};

}  // namespace dsopp::energy::problem

#endif  // DSOPP_ENERGY_PROBLEMS_PHOTOMETRIC_BUNDLE_ADJUSTMENT_PROBLEM_HPP_
