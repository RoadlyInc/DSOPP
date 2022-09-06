#ifndef DSOPP_ENERGY_PROBLEMS_CERES_PHOTOMETRIC_BUNDLE_ADJUSTMENT_ADD_BLOCKS_HPP_
#define DSOPP_ENERGY_PROBLEMS_CERES_PHOTOMETRIC_BUNDLE_ADJUSTMENT_ADD_BLOCKS_HPP_

#include <ceres/problem.h>

#include "energy/motion/motion.hpp"
#include "energy/problems/cost_functors/affine_brightness_regularization_cost_functor.hpp"
#include "energy/problems/cost_functors/bundle_adjustment_photometric_cost_functor.hpp"
#include "energy/problems/cost_functors/bundle_adjustment_photometric_cost_functor_analytic.hpp"
#include "energy/problems/photometric_bundle_adjustment/local_frame.hpp"
#include "energy/problems/photometric_bundle_adjustment/state_priors.hpp"

namespace dsopp::energy::problem {

template <energy::motion::Motion Motion, template <int> typename Grid2D, model::Model Model, int PatternSize, int C>
void addResidualBlock(ceres::Problem &problem,
                      LocalFrame<double, Motion, Model, PatternSize, Grid2D, C> &reference_frame,
                      LocalFrame<double, Motion, Model, PatternSize, Grid2D, C> &target_frame,
                      typename LocalFrame<double, Motion, Model, PatternSize, Grid2D, C>::Landmark &landmark,
                      size_t sensor_id, ceres::LossFunction *loss_function,
                      const ResidualPoint<double, Motion, PatternSize, C> &residual_point,
                      const bool use_analitic_diff) {
  if (residual_point.connection_status == track::PointConnectionStatus::kOk) {
    if (use_analitic_diff) {
      using CostFunctor = cost_functors::BundleAdjustmentPhotometricCostFunctorAnalytic<Motion, PatternSize, C>;
      auto *cost_function = new CostFunctor(residual_point);
      problem.AddResidualBlock(cost_function, loss_function, reference_frame.state_eps.data(),
                               target_frame.state_eps.data(), &landmark.idepth);
    } else {
      using CostFunctor =
          cost_functors::BundleAdjustmentPhotometricCostFunctor<Motion, Grid2D<C>, Model, PatternSize, C>;
      auto *cost_function = new ceres::AutoDiffCostFunction<CostFunctor, CostFunctor::residuals_num, Motion::DoF + 2,
                                                            Motion::DoF + 2, 1, Model::DoF, Model::DoF>(
          new CostFunctor(reference_frame.T_w_agent_linearization_point, target_frame.T_w_agent_linearization_point,
                          reference_frame.exposure_time, reference_frame.affine_brightness0, target_frame.exposure_time,
                          target_frame.affine_brightness0, *target_frame.grids.at(sensor_id),
                          landmark.reference_pattern, landmark.patch, target_frame.masks.at(sensor_id),
                          reference_frame.model.image_size(), target_frame.model.image_size()));
      problem.AddResidualBlock(cost_function, loss_function, reference_frame.state_eps.data(),
                               target_frame.state_eps.data(), &landmark.idepth,
                               reference_frame.intrinsic_parameters.data(), target_frame.intrinsic_parameters.data());
    }
  }
}

template <energy::motion::Motion Motion, model::Model Model, int PatternSize, template <int> typename Grid2D, int C>
void addConstraintToProblem(LocalFrame<double, Motion, Model, PatternSize, Grid2D, C> &reference_frame,
                            LocalFrame<double, Motion, Model, PatternSize, Grid2D, C> &target_frame,
                            ceres::Problem &problem, size_t sensor_id, ceres::LossFunction *loss_function,
                            const bool use_analitic_diff) {
  if (target_frame.is_marginalized) {
    return;
  }
  auto &landmarks_frame = reference_frame.active_landmarks;
  for (auto &[sensor, landmarks] : landmarks_frame) {
    if (not reference_frame.residuals[{sensor, sensor}].contains(target_frame.id)) {
      reference_frame.residuals[{sensor, sensor}][target_frame.id].resize(landmarks.size());
    }
    const auto &residuals = reference_frame.residuals[{sensor, sensor}].at(target_frame.id);
    for (size_t landmark_index = 0; landmark_index < residuals.size(); landmark_index++) {
      auto &landmark = landmarks[landmark_index];
      if (not landmark.is_outlier and
          residuals[landmark_index].connection_status == track::PointConnectionStatus::kOk) {
        addResidualBlock<Motion, Grid2D, Model, PatternSize, C>(problem, reference_frame, target_frame, landmark,
                                                                sensor_id, loss_function, residuals[landmark_index],
                                                                use_analitic_diff);
      }
    }
  }
}

template <energy::motion::Motion Motion, energy::model::Model Model, int PatternSize, template <int> typename Grid2D,
          int C>
void addLandmarks(
    ceres::Problem &problem,
    std::map<size_t, std::vector<typename LocalFrame<double, Motion, Model, PatternSize, Grid2D, C>::Landmark>>
        &landmarks_frame,
    ceres::ParameterBlockOrdering *ordering) {
  for (auto &[sensor, landmarks] : landmarks_frame) {
    for (auto &landmark : landmarks) {
      problem.AddParameterBlock(&landmark.idepth, 1);
      ordering->AddElementToGroup(&landmark.idepth, 0);
    }
  }
}
template <energy::motion::Motion Motion, energy::model::Model Model, int PatternSize, template <int> typename Grid2D,
          int C, bool ForceAll>
void setLandmarksConst(
    ceres::Problem &problem,
    std::map<size_t, std::vector<typename LocalFrame<double, Motion, Model, PatternSize, Grid2D, C>::Landmark>>
        &landmarks_frame) {
  for (auto &[sensor, landmarks] : landmarks_frame) {
    for (auto &landmark : landmarks) {
      if (ForceAll or landmark.is_marginalized) {
        problem.SetParameterBlockConstant(&landmark.idepth);
      }
    }
  }
}

template <energy::motion::Motion Motion>
void addAffineBrightnessRegularization(ceres::Problem &problem, Eigen::Vector2d &affine_brightness0,
                                       Eigen::Vector<double, Motion::DoF + 2> &state_eps,
                                       const Eigen::Vector2d &affineBrightnessRegularizer) {
  using CostFunctor = cost_functors::AffineBrightnessRegularizationCostFunctor<Motion::DoF>;
  auto *cost_functor = new CostFunctor(affineBrightnessRegularizer.cwiseSqrt(), affine_brightness0);
  ceres::CostFunction *cost_function =
      new ceres::AutoDiffCostFunction<CostFunctor, CostFunctor::residuals_num, Motion::DoF + 2>(cost_functor);
  problem.AddResidualBlock(cost_function, nullptr, state_eps.data());
}

template <energy::motion::Motion Motion, model::Model Model, int PatternSize, template <int> typename Grid2D,
          bool OPTIMIZE_POSES, bool OPTIMIZE_IDEPTHS, bool OPTIMIZE_CAMERA, int C>
void addFramesToProblem(std::deque<std::unique_ptr<LocalFrame<double, Motion, Model, PatternSize, Grid2D, C>>> &frames,
                        size_t sensor_id, ceres::Problem &problem, ceres::LossFunction *loss_function,
                        ceres::ParameterBlockOrdering *ordering, const Eigen::Vector2d &affineBrightnessRegularizer,
                        const bool use_analitic_diff) {
  const bool kFixFocal = false;
  const bool kFixCenter = true;
  const bool kFixModelSpecific = false;
  auto constant_parameters = Model::constantParameterSet(kFixFocal, kFixCenter, kFixModelSpecific);

  for (size_t idx = 0; idx < frames.size(); ++idx) {
    auto &frame = *frames[idx];
    if (!OPTIMIZE_CAMERA || use_analitic_diff || constant_parameters.size() == Model::DoF) {
      problem.AddParameterBlock(frame.intrinsic_parameters.data(), Model::DoF);
      problem.SetParameterBlockConstant(frame.intrinsic_parameters.data());
    } else {
      if (!constant_parameters.empty()) {
        ceres::LocalParameterization *camera_parameterization =
            new ceres::SubsetParameterization(Model::DoF, constant_parameters);

        problem.AddParameterBlock(frame.intrinsic_parameters.data(), Model::DoF, camera_parameterization);
      }
    }
  }
  for (size_t idx = 0; idx < frames.size(); ++idx) {
    auto &frame = *frames[idx];
    ordering->AddElementToGroup(frame.intrinsic_parameters.data(), 1);
    FrameParameterization frame_parameterization = frame.frame_parameterization;

    problem.AddParameterBlock(frame.state_eps.data(), Motion::DoF + 2);
    ordering->AddElementToGroup(frame.state_eps.data(), 1);
    if (not OPTIMIZE_POSES or frame.is_marginalized or frame_parameterization == FrameParameterization::kFixed) {
      problem.SetParameterBlockConstant(frame.state_eps.data());
    } else {
      addAffineBrightnessRegularization<Motion>(problem, frame.affine_brightness0, frame.state_eps,
                                                affineBrightnessRegularizer);
      hessian_prior::MotionPrior<Motion>::addCeresPrior(problem, frame.state_eps);
      if (idx != frames.size() - 1) {
        auto &target_frame = *frames[idx + 1];
        hessian_prior::MotionPrior<Motion>::addCeresRelativeMotionPrior(
            problem, frame.state_eps, target_frame.state_eps, frame.model.shutterTime(), frame.model.image_size(),
            (target_frame.timestamp - frame.timestamp));
      }
    }
  }

  for (size_t target_idx = 0; target_idx < frames.size(); ++target_idx) {
    auto &target_frame = *frames[target_idx];

    if constexpr (OPTIMIZE_IDEPTHS) {
      addLandmarks<Motion, Model, PatternSize, Grid2D, C>(problem, target_frame.active_landmarks, ordering);
      setLandmarksConst<Motion, Model, PatternSize, Grid2D, C, false>(problem, target_frame.active_landmarks);
    } else {
      addLandmarks<Motion, Model, PatternSize, Grid2D, C>(problem, target_frame.active_landmarks, ordering);
      setLandmarksConst<Motion, Model, PatternSize, Grid2D, C, true>(problem, target_frame.active_landmarks);
    }
    for (size_t reference_idx = 0; reference_idx < target_idx; reference_idx++) {
      auto &reference_frame = *frames[reference_idx];

      addConstraintToProblem<Motion, Model, PatternSize, Grid2D, C>(reference_frame, target_frame, problem, sensor_id,
                                                                    loss_function, use_analitic_diff);
      addConstraintToProblem<Motion, Model, PatternSize, Grid2D, C>(target_frame, reference_frame, problem, sensor_id,
                                                                    loss_function, use_analitic_diff);
    }
  }
}

}  // namespace dsopp::energy::problem

#endif  // DSOPP_ENERGY_PROBLEMS_CERES_PHOTOMETRIC_BUNDLE_ADJUSTMENT_ADD_BLOCKS_HPP_
