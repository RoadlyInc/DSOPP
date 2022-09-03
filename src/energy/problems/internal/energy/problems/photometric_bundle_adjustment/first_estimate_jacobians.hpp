#ifndef DSOPP_ENERGY_PROBLEM_FIRST_ESTIMATE_JACOBIANS_HPP_
#define DSOPP_ENERGY_PROBLEM_FIRST_ESTIMATE_JACOBIANS_HPP_

#include <deque>
#include <memory>

#include <tbb/parallel_for.h>

#include "energy/motion/motion.hpp"
#include "energy/problems/photometric_bundle_adjustment/local_frame.hpp"
#include "energy/projector/camera_reproject.hpp"

namespace dsopp::energy::problem {
template <typename Scalar, energy::motion::Motion Motion, typename Model, int PatternSize,
          template <int> typename Grid2D, int C>
void firstEstimateJacobians_(
    std::deque<std::unique_ptr<LocalFrame<Scalar, Motion, Model, PatternSize, Grid2D, C>>> &frames) {
  using MotionCasted = typename Motion::template CastT<Scalar>;
  tbb::parallel_for(tbb::blocked_range<size_t>(0, frames.size()), [&](auto &frame_r) {
    for (size_t frame_id = frame_r.begin(); frame_id != frame_r.end(); ++frame_id) {
      auto &reference_frame = frames[frame_id];

      for (auto &target_frame : frames) {
        if (reference_frame->id == target_frame->id) {
          continue;
        }

        typename MotionCasted::Product t_t_r0 =
            target_frame->T_w_agent_linearization_point.inverse() * reference_frame->T_w_agent_linearization_point;

        reprojection::ArrayReprojector<Scalar, Model, typename MotionCasted::Product> reprojector(
            reference_frame->model, target_frame->model, t_t_r0.template cast<Scalar>());

        const Eigen::Vector2<Scalar> reference_affine_brightness = reference_frame->affine_brightness0;
        const Eigen::Vector2<Scalar> target_affine_brightness = target_frame->affine_brightness0;
        const Scalar brightness_change_scale = (target_frame->exposure_time / reference_frame->exposure_time) *
                                               std::exp(target_affine_brightness[0] - reference_affine_brightness[0]);

        for (const auto &sensor_1 : reference_frame->sensors()) {
          for (const auto &sensor_2 : reference_frame->sensors()) {
            if (not reference_frame->residuals[{sensor_1, sensor_2}].contains(target_frame->id)) {
              continue;
            }
            auto &residuals = reference_frame->residuals[{sensor_1, sensor_2}].at(target_frame->id);
            auto &landmarks = reference_frame->active_landmarks[sensor_1];
            tbb::parallel_for(tbb::blocked_range<size_t>(0, landmarks.size()), [&](auto r) {
              for (auto landmark_index = r.begin(); landmark_index != r.end(); ++landmark_index) {
                auto &landmark = landmarks[landmark_index];
                if (landmark.is_marginalized && !landmark.to_marginalize) continue;
                auto &residual = residuals[landmark_index];
                Eigen::Matrix<Scalar, 2, PatternSize> target_pattern;
                residual.reprojection_jacobians_valid = reprojector.reprojectPattern(
                    landmark.reference_pattern, landmark.idepth, target_pattern, residual.d_u_idepth,
                    residual.d_v_idepth, residual.d_u_tReferenceTarget, residual.d_v_tReferenceTarget);
                // we dont need to check for mask here, as we only evaluate derivatives of projections

                const Eigen::Matrix<Scalar, PatternSize, C, PatchStorageOrder<C>> corrected_reference_intensities =
                    brightness_change_scale *
                    (landmark.patch - Eigen::Matrix<Scalar, PatternSize, C, PatchStorageOrder<C>>::Constant(
                                          reference_affine_brightness[1]));

                landmark.corrected_intensities = corrected_reference_intensities;
                residual.brightness_change_scale = brightness_change_scale;
              }
            });
          }
        }
      }
    }
  });
}

}  // namespace dsopp::energy::problem

#endif  // DSOPP_ENERGY_PROBLEM_FIRST_ESTIMATE_JACOBIANS_HPP_
