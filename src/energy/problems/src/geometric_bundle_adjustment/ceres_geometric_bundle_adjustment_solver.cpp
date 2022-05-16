#include "energy/motion/se3_motion.hpp"
#include "energy/problems/geometric_bundle_adjustment/ceres_geometric_bundle_adjustment.hpp"

#include <ceres/ceres.h>
#include <ceres/local_parameterization.h>
#include <ceres/ordered_groups.h>
#include <ceres/types.h>
#include <glog/logging.h>

#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/camera_model/pinhole/simple_radial.hpp"
#include "energy/motion/local_parameterization_se3.hpp"
#include "energy/problems/cost_functors/bundle_adjustment_geometric_cost_functor.hpp"
#include "energy/problems/local_parameterization_s2.hpp"

namespace dsopp::energy::problem::geometric_bundle_adjustment {
template <model::Model Model>
CeresGeometricBundleAdjustmentSolver<Model>::CeresGeometricBundleAdjustmentSolver(
    const Eigen::Vector<Precision, 2> &image_size, const Eigen::Vector<Precision, Model::DoF> &camera_intrinsics,
    int number_of_threads, double loss_threshold, double max_solver_time)
    : camera_intrinsics_(camera_intrinsics.template cast<double>()),
      image_size_(image_size),
      number_of_threads_(number_of_threads),
      loss_threshold_(loss_threshold),
      max_solver_time_(max_solver_time) {}

template <model::Model Model>
const Eigen::Vector<double, Model::DoF> &CeresGeometricBundleAdjustmentSolver<Model>::camera_intrinsics() const {
  return camera_intrinsics_;
}

template <model::Model Model>
bool CeresGeometricBundleAdjustmentSolver<Model>::solve(std::vector<LocalFrame> &local_frames,
                                                        ParameterParameterization parameter_parameterization) {
  ceres::Problem problem;
  auto ordering = new ceres::ParameterBlockOrdering();
  if (local_frames.size() < 2) {
    return false;
  }

  /** fixing first pose and translation magntitude on second frame to decrease degree of freedom
   * but this is arguable (https://www.doc.ic.ac.uk/~ajd/Publications/Strasdat-H-2012-PhD-Thesis.pdf)
   */
  problem.AddParameterBlock(local_frames[0].t_agent_world.data(), motion::SE3<double>::num_parameters);
  problem.SetParameterBlockConstant(local_frames[0].t_agent_world.data());
  problem.AddParameterBlock(local_frames[1].t_agent_world.data(), motion::SE3<double>::num_parameters,
                            new ceres::ProductParameterization(new ceres::EigenQuaternionParameterization(),
                                                               new ceres::HomogeneousVectorParameterization(3)));

  for (size_t i = 2; i < local_frames.size(); ++i) {
    problem.AddParameterBlock(local_frames[i].t_agent_world.data(), motion::SE3<double>::num_parameters,
                              new energy::motion::LocalParameterizationSE3());
  }

  for (auto &frame : local_frames) {
    ordering->AddElementToGroup(frame.t_agent_world.data(), 1);
    if (parameter_parameterization.fix_poses) {
      problem.SetParameterBlockConstant(frame.t_agent_world.data());
    }
  }

  problem.AddParameterBlock(camera_intrinsics_.data(), Model::DoF);
  ordering->AddElementToGroup(camera_intrinsics_.data(), 1);

  auto constant_parameters =
      Model::constantParameterSet(parameter_parameterization.fix_focal, parameter_parameterization.fix_center,
                                  parameter_parameterization.fix_model_specific);

  if (constant_parameters.size() == Model::DoF) {
    problem.AddParameterBlock(camera_intrinsics_.data(), Model::DoF);
    problem.SetParameterBlockConstant(camera_intrinsics_.data());
  } else if (!constant_parameters.empty()) {
    problem.AddParameterBlock(camera_intrinsics_.data(), Model::DoF,
                              new ceres::SubsetParameterization(Model::DoF, constant_parameters));
  }

  typename Model::template CastT<double> camera(image_size_, camera_intrinsics_);
  using Cost = energy::problem::cost_functors::BundleAdjustmentGeometricCostFunctor<Model>;
  for (auto &local_frame : local_frames) {
    for (size_t i = 0; i < local_frame.size(); ++i) {
      auto [point_2d, point_3d] = local_frame.point(i);
      auto *cost = new ceres::AutoDiffCostFunction<Cost, Cost::residuals_num, motion::SE3<double>::num_parameters, 3,
                                                   Model::DoF>(new Cost(image_size_, point_2d));

      problem.AddResidualBlock(cost, new ceres::HuberLoss(loss_threshold_), local_frame.t_agent_world.data(),
                               point_3d.data(), camera_intrinsics_.data());
      ordering->AddElementToGroup(point_3d.data(), 0);
      if (parameter_parameterization.fix_landmarks) {
        problem.SetParameterBlockConstant(point_3d.data());
      }
    }
  }

  ceres::Solver::Options options;
  options.num_threads = static_cast<int>(number_of_threads_);
  options.linear_solver_ordering.reset(ordering);
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.logging_type = ceres::SILENT;
  options.max_num_iterations = 1000;
  options.function_tolerance = 1e-10;
  options.parameter_tolerance = 1e-10;
  options.max_solver_time_in_seconds = max_solver_time_;
  ceres::Solver::Summary summary;
  Solve(options, &problem, &summary);
  VLOG(1) << summary.FullReport();

  return summary.IsSolutionUsable();
}

#define CeresGeometricBundleAdjustmentSolverInstantiation(Model) \
  template class CeresGeometricBundleAdjustmentSolver<model::Model<Precision>>
CeresGeometricBundleAdjustmentSolverInstantiation(SimpleRadialCamera);
CeresGeometricBundleAdjustmentSolverInstantiation(PinholeCamera);
#undef CeresGeometricBundleAdjustmentSolverInstantiation

}  // namespace dsopp::energy::problem::geometric_bundle_adjustment
