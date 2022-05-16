#ifndef DSOPP_ENERGY_PROBLEM_HESSIAN_BLOCK_EVALUATION_HPP_
#define DSOPP_ENERGY_PROBLEM_HESSIAN_BLOCK_EVALUATION_HPP_

#include <mutex>

#include <tbb/parallel_for.h>
#include <Eigen/Dense>

#include "common/settings.hpp"
#include "energy/motion/motion.hpp"
#include "energy/normal_linear_system.hpp"
#include "energy/problems/photometric_bundle_adjustment/local_frame.hpp"
#include "matrix_accumulator.hpp"

/*
 * Code in this file solves photometric bundle adjustment with Gauss-Newton(or Levenberg–Marquardt) algorithm. The main
 * functionality of the following code is to construct a matrix H = J' * J and b = J' * r for sparse data this->frames_,
 * where r is a vector of residuals, and J is a jacobi matrix.
 *
 * Let J = [Jp Jd] br a block matrix, where Jp is a jacobian of residuals with respect to poses, and Jd is a jacobian of
 * residuals with respect to inverse depths.
 * b is therefore = [b_p b_d]= [Jp'r Jd'r]
 * of residuals with respect to inverse depths of landmarks H matrix has a special sparsity pattern in the photometric
 * bundle adjustment problem: H = [H_pp  H_pd
 *                                 H_pd' H_dd]
 *
 *
 * H_pp is a dense matrix reflecting interconnection of given poses (H_pp = Jp' * Jp);
 * H_dd is a diagonal matrix reflecting interconnection of depths (H_dd = Jd' * Jd);
 * H_pd is a dense matrix reflection interconnection of poses and depth (H_pd = Jp' * Jd);
 *
 * We use schur complement to solve linear system Hx = b:
 * http://ceres-solver.org/nnls_solving.html#dense-schur-sparse-schur
 * Our linear system is then reduced to the size of H_pp.
 */

namespace dsopp::energy::problem {
template <bool FOR_MARGINALIZED, energy::motion::Motion Motion, typename Model, int PatternSize,
          template <int> typename Grid2D, int C, typename EigenDerivedA, typename EigenDerivedB, typename EigenDerivedC,
          typename EigenDerivedD, typename EigenDerivedE>
void evaluateLinearSystemPosePoseBlock(
    const LocalFrame<Precision, Motion, Model, PatternSize, Grid2D, C> &reference_frame,
    const LocalFrame<Precision, Motion, Model, PatternSize, Grid2D, C> &target_frame, size_t sensor_id,
    Eigen::MatrixBase<EigenDerivedA> &hessian_reference_reference,
    Eigen::MatrixBase<EigenDerivedB> &hessian_reference_target, Eigen::MatrixBase<EigenDerivedC> &hessian_target_target,
    Eigen::MatrixBase<EigenDerivedD> &b_reference, Eigen::MatrixBase<EigenDerivedE> &b_target) noexcept {
  const int kRows = EigenDerivedA::RowsAtCompileTime;
  const int kCols = EigenDerivedA::ColsAtCompileTime;
  MatrixAccumulator<Precision, kRows, kCols> hessian_reference_reference_accumulator;
  MatrixAccumulator<Precision, kRows, kCols> hessian_reference_target_accumulator;
  MatrixAccumulator<Precision, kRows, kCols> hessian_target_target_accumulator;
  MatrixAccumulator<Precision, kRows, 1> b_reference_accumulator;
  MatrixAccumulator<Precision, kRows, 1> b_target_accumulator;

  auto &reference_residuals = reference_frame.residuals.at({sensor_id, sensor_id});
  if (!reference_residuals.contains(target_frame.id)) {
    return;
  }
  const auto &residuals = reference_residuals.at(target_frame.id);
  const auto &landmarks = reference_frame.active_landmarks.at(sensor_id);

  auto landmark_iter = landmarks.begin();
  const auto landmark_end = landmarks.end();
  CHECK_EQ(landmarks.size(), residuals.size());
  auto residuals_iter = residuals.begin();
  for (; landmark_iter != landmark_end; ++landmark_iter, ++residuals_iter) {
    const auto &landmark = *landmark_iter;
    if constexpr (FOR_MARGINALIZED) {
      if (!landmark.to_marginalize) continue;
    } else {
      if (landmark.is_marginalized) continue;
    }
    const auto &residual = *residuals_iter;
    hessian_reference_reference_accumulator +=
        residual.huber_weight * residual.d_reference_state_eps.transpose().lazyProduct(residual.d_reference_state_eps);
    hessian_target_target_accumulator +=
        residual.huber_weight * residual.d_target_state_eps.transpose().lazyProduct(residual.d_target_state_eps);
    hessian_reference_target_accumulator +=
        residual.huber_weight * residual.d_reference_state_eps.transpose().lazyProduct(residual.d_target_state_eps);
    b_reference_accumulator +=
        residual.huber_weight * residual.d_reference_state_eps.transpose().lazyProduct(residual.residuals);
    b_target_accumulator +=
        residual.huber_weight * residual.d_target_state_eps.transpose().lazyProduct(residual.residuals);
  }
  hessian_reference_reference = hessian_reference_reference_accumulator.value();
  hessian_target_target = hessian_target_target_accumulator.value();
  hessian_reference_target = hessian_reference_target_accumulator.value();
  b_reference = b_reference_accumulator.value();
  b_target = b_target_accumulator.value();
}
template <bool FOR_MARGINALIZED = false, energy::motion::Motion Motion, typename Model, int PatternSize,
          template <int> typename Grid2D, int C>
/**
 * calculates H_pp and b_p matrices
 */
void evaluateLinearSystemPosePose(
    const std::deque<std::unique_ptr<LocalFrame<Precision, Motion, Model, PatternSize, Grid2D, C>>> &frames,
    size_t sensor_id, NormalLinearSystem<> &system_pose) {
  const int kBlockSize = Motion::DoF + 2;

  std::mutex mutex;
  tbb::parallel_for(tbb::blocked_range<size_t>(0, frames.size()), [&](auto r) {
    Eigen::Matrix<Precision, kBlockSize, kBlockSize> hessian_reference_reference;
    Eigen::Matrix<Precision, kBlockSize, kBlockSize> hessian_reference_target;
    Eigen::Matrix<Precision, kBlockSize, kBlockSize> hessian_target_target;
    Eigen::Vector<Precision, kBlockSize> b_reference;
    Eigen::Vector<Precision, kBlockSize> b_target;
    for (auto reference_frame_index = r.begin(); reference_frame_index != r.end(); ++reference_frame_index) {
      const auto &reference_frame = frames[reference_frame_index];
      for (size_t target_frame_index = 0; target_frame_index < frames.size(); ++target_frame_index) {
        if (reference_frame_index == target_frame_index) {
          continue;
        }
        const auto &target_frame = frames[target_frame_index];
        evaluateLinearSystemPosePoseBlock<FOR_MARGINALIZED>(*reference_frame, *target_frame, sensor_id,
                                                            hessian_reference_reference, hessian_reference_target,
                                                            hessian_target_target, b_reference, b_target);
        {
          std::lock_guard guard(mutex);

          system_pose.H
              .template block<kBlockSize, kBlockSize>(kBlockSize * static_cast<int>(reference_frame_index),
                                                      kBlockSize * static_cast<int>(reference_frame_index))
              .noalias() += hessian_reference_reference;
          system_pose.H
              .template block<kBlockSize, kBlockSize>(kBlockSize * static_cast<int>(reference_frame_index),
                                                      kBlockSize * static_cast<int>(target_frame_index))
              .noalias() = hessian_reference_target;

          system_pose.H
              .template block<kBlockSize, kBlockSize>(kBlockSize * static_cast<int>(target_frame_index),
                                                      kBlockSize * static_cast<int>(target_frame_index))
              .noalias() += hessian_target_target;

          system_pose.b.template segment<kBlockSize>(kBlockSize * static_cast<int>(reference_frame_index)).noalias() +=
              b_reference;
          system_pose.b.template segment<kBlockSize>(kBlockSize * static_cast<int>(target_frame_index)).noalias() +=
              b_target;
        }
      }

      // only first frame can be fixed as for now
      CHECK(reference_frame->frame_parameterization == FrameParameterization::kFree or reference_frame_index == 0);
    }
  });

  tbb::parallel_for(tbb::blocked_range<size_t>(0, frames.size()), [&](auto r) {
    for (auto reference_frame_index = r.begin(); reference_frame_index != r.end(); ++reference_frame_index) {
      int reference_block_index = kBlockSize * static_cast<int>(reference_frame_index);
      system_pose.H.template block<kBlockSize, kBlockSize>(reference_block_index, reference_block_index) =
          system_pose.H.template block<kBlockSize, kBlockSize>(reference_block_index, reference_block_index)
              .template selfadjointView<Eigen::Lower>();

      for (size_t target_frame_index = reference_frame_index + 1; target_frame_index < frames.size();
           ++target_frame_index) {
        int target_block_index = kBlockSize * static_cast<int>(target_frame_index);
        system_pose.H.template block<kBlockSize, kBlockSize>(reference_block_index, target_block_index) +=
            system_pose.H.template block<kBlockSize, kBlockSize>(target_block_index, reference_block_index).transpose();
        system_pose.H.template block<kBlockSize, kBlockSize>(target_block_index, reference_block_index) =
            system_pose.H.template block<kBlockSize, kBlockSize>(reference_block_index, target_block_index).transpose();
      }
    }
  });
}
/**
 * calculates schur complement of of H_dd:
 * H_pd * inv(H_dd) * H_pd' and H_pd * inv(H_dd) * b_d matrices
 */
template <bool FOR_MARGINALIZED = false, energy::motion::Motion Motion, typename Model, int PatternSize,
          template <int> typename Grid2D, int C>
void evaluateLinearSystemPoseDepthSchurComplement(
    std::deque<std::unique_ptr<LocalFrame<Precision, Motion, Model, PatternSize, Grid2D, C>>> &frames,
    const size_t sensor_id, NormalLinearSystem<> &system_schur) noexcept {
  const int kBlockSize = Motion::DoF + 2;
  const Precision kScaleNullspaceRegularizer = 1e8 * C;

  system_schur.setZero();
  std::mutex mutex;
  tbb::parallel_for(tbb::blocked_range<size_t>(0, frames.size()), [&](auto r) {
    MatrixAccumulator<Precision> hessian(system_schur.H.rows(), system_schur.H.cols());
    MatrixAccumulator<Precision, Eigen::Dynamic, 1> b(system_schur.b.rows(), 1);

    Eigen::VectorX<Precision> hessian_poses_idepth_block(static_cast<int>(kBlockSize * frames.size()));
    for (auto reference_frame_index = r.begin(); reference_frame_index != r.end(); ++reference_frame_index) {
      auto &reference_frame = *frames[reference_frame_index];
      const auto &residuals = reference_frame.residuals.at({sensor_id, sensor_id});
      auto &landmarks = reference_frame.active_landmarks.at(sensor_id);
      for (size_t landmark_index = 0; landmark_index < landmarks.size(); landmark_index++) {
        auto &landmark = landmarks[landmark_index];
        if constexpr (FOR_MARGINALIZED) {
          if (!landmark.to_marginalize) continue;
        } else {
          if (landmark.is_marginalized) continue;
        }
        hessian_poses_idepth_block.setZero();
        Precision b_idepth_block = 0;
        Precision hessian_idepth_idepth = 0;
        for (size_t target_frame_index = 0; target_frame_index < frames.size(); ++target_frame_index) {
          if (reference_frame_index == target_frame_index) {
            continue;
          }
          const auto &target_frame = *frames[target_frame_index];
          const auto &residual = residuals.at(target_frame.id)[landmark_index];
          hessian_poses_idepth_block.segment<kBlockSize>(kBlockSize * static_cast<int>(reference_frame_index))
              .noalias() +=
              residual.huber_weight * residual.d_reference_state_eps.transpose().lazyProduct(residual.d_idepth);
          hessian_poses_idepth_block.segment<kBlockSize>(kBlockSize * static_cast<int>(target_frame_index)).noalias() +=
              residual.huber_weight * residual.d_target_state_eps.transpose().lazyProduct(residual.d_idepth);
          hessian_idepth_idepth += residual.huber_weight * residual.d_idepth.squaredNorm();
          b_idepth_block +=
              (residual.huber_weight * residual.d_idepth.transpose().lazyProduct(residual.residuals))(0, 0);
        }
        landmark.b_idepth_block = b_idepth_block;
        landmark.hessian_poses_idepth_block = hessian_poses_idepth_block;
        const Precision kIdepthNullSpaceThreshold = 1e-15_p * C;
        if (hessian_idepth_idepth > kIdepthNullSpaceThreshold) {
          if (FOR_MARGINALIZED && reference_frame.frame_parameterization == FrameParameterization::kFixed) {
            hessian_idepth_idepth += kScaleNullspaceRegularizer;
          }
          landmark.inv_hessian_idepth_idepth = 1.0_p / (hessian_idepth_idepth);
          landmark.ill_conditioned = false;
          b += (landmark.inv_hessian_idepth_idepth * b_idepth_block) * hessian_poses_idepth_block;
          hessian += landmark.inv_hessian_idepth_idepth *
                     hessian_poses_idepth_block.lazyProduct(hessian_poses_idepth_block.transpose());
        } else {
          landmark.ill_conditioned = true;
        }
      }
    }
    {
      std::lock_guard guard(mutex);
      system_schur.H += hessian.value();
      system_schur.b += b.value();
    }
  });
}

template <energy::motion::Motion Motion, typename Model, int PatternSize, template <int> typename Grid2D, int C,
          typename EigenDerivedA>
void calculateIdepths(std::deque<std::unique_ptr<LocalFrame<Precision, Motion, Model, PatternSize, Grid2D, C>>> &frames,
                      size_t sensor_id, Eigen::MatrixBase<EigenDerivedA> &step_poses,
                      Precision levenberg_marquardt_lambda) {
  const Precision kLevenbergMarquardtLambdaInversed = 1.0_p / (1.0_p + levenberg_marquardt_lambda);

  for (size_t reference_frame_index = 0; reference_frame_index < frames.size(); ++reference_frame_index) {
    auto &reference_frame = *frames[reference_frame_index];
    auto &active_landmarks = reference_frame.active_landmarks[sensor_id];
    tbb::parallel_for(tbb::blocked_range<size_t>(0, active_landmarks.size()), [&](auto r) {
      for (auto landmark_i = r.begin(); landmark_i != r.end(); ++landmark_i) {
        auto &landmark = reference_frame.active_landmarks[sensor_id][landmark_i];
        if (landmark.is_marginalized) {
          continue;
        }

        if (not landmark.ill_conditioned) {
          Precision step = (landmark.b_idepth_block - landmark.hessian_poses_idepth_block.transpose() * step_poses) *
                           kLevenbergMarquardtLambdaInversed * landmark.inv_hessian_idepth_idepth;
          landmark.idepth_step = -step;
        }
      }
    });
  }
}

}  // namespace dsopp::energy::problem

#endif  // DSOPP_ENERGY_PROBLEM_HESSIAN_BLOCK_EVALUATION_HPP_
