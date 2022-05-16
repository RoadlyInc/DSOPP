#ifndef DSOPP_COVARIANCE_MATRICES_OF_RELATIVE_POSES_HPP
#define DSOPP_COVARIANCE_MATRICES_OF_RELATIVE_POSES_HPP

#include <tbb/blocked_range2d.h>
#include <tbb/parallel_for.h>

#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"
#include "energy/problems/photometric_bundle_adjustment/local_frame.hpp"
#include "measures/similarity_measure.hpp"
#include "sensors/camera/camera.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"

namespace dsopp {
namespace energy {
namespace problem {
/**
 * Estimate relative transformations uncertainty using covariance matrices between poses
 * @param frames local frames to fill covariance matrices of relative poses
 * @param covariance_matrix_pose_pose covariance matrices between poses */
template <typename Scalar, energy::motion::Motion Motion, energy::model::Model Model, int PatternSize,
          template <int> typename Grid2D, int C>
void covarianceMatricesOfRelativePoses(
    std::deque<std::unique_ptr<LocalFrame<Scalar, Motion, Model, PatternSize, Grid2D, C>>> &frames,
    const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> &covariance_matrix_pose_pose) {
  const int kBlockSize = Motion::DoF + 2;

  for (size_t reference_idx = 0; reference_idx < frames.size(); ++reference_idx) {
    auto &reference_frame = *frames[reference_idx];
    for (size_t target_idx = 0; target_idx < frames.size(); ++target_idx) {
      auto &target_frame = *frames[target_idx];
      if (reference_frame.id == target_frame.id) {
        continue;
      }
      reference_frame.covariance_matrices[target_frame.id];
    }
  }

  tbb::parallel_for(tbb::blocked_range2d<size_t, size_t>(0, frames.size(), 0, frames.size()), [&](auto &r) {
    for (size_t reference_idx = r.rows().begin(); reference_idx != r.rows().end(); ++reference_idx) {
      auto &reference_frame = *frames[reference_idx];
      for (size_t target_idx = r.cols().begin(); target_idx != r.cols().end(); ++target_idx) {
        auto &target_frame = *frames[target_idx];
        if (reference_frame.id == target_frame.id) {
          continue;
        }
        Eigen::Matrix<Scalar, Motion::DoF, Motion::DoF> sigma_rr =
            covariance_matrix_pose_pose.template block<Motion::DoF, Motion::DoF>(
                static_cast<long>(reference_idx * kBlockSize), static_cast<long>(reference_idx * kBlockSize));
        Eigen::Matrix<Scalar, Motion::DoF, Motion::DoF> sigma_tt =
            covariance_matrix_pose_pose.template block<Motion::DoF, Motion::DoF>(
                static_cast<long>(target_idx * kBlockSize), static_cast<long>(target_idx * kBlockSize));
        Eigen::Matrix<Scalar, Motion::DoF, Motion::DoF> sigma_rt =
            covariance_matrix_pose_pose.template block<Motion::DoF, Motion::DoF>(
                static_cast<long>(reference_idx * kBlockSize), static_cast<long>(target_idx * kBlockSize));
        reference_frame.covariance_matrices[target_frame.id] =
            Motion::template CastT<Scalar>::Product::relativeTransformationUncertainty(
                reference_frame.tWorldAgent(), target_frame.tWorldAgent(), sigma_rr, sigma_tt, sigma_rt);
      }
    }
  });
}

}  // namespace problem
}  // namespace energy
}  // namespace dsopp

#endif  // DSOPP_COVARIANCE_MATRICES_OF_RELATIVE_POSES_HPP
