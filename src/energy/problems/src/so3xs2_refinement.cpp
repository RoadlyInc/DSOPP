
#include "energy/problems/so3xs2_refinement.hpp"

#include <glog/logging.h>
#include "energy/problems/cost_functors/sampson_distance_cost.hpp"
#include "energy/problems/local_parameterization_s2.hpp"

namespace dsopp::energy::problem {

template <bool OPTIMIZE_FOCAL, bool OPTIMIZE_POSE>
void refineSO3xS2(typename std::conditional<OPTIMIZE_FOCAL, double &, double>::type focal, Sophus::SE3d &t_t_r,
                  const std::vector<Eigen::Vector2d> &points_reference,
                  const std::vector<Eigen::Vector2d> &points_target, double threshold, const int number_of_threads) {
  CHECK_EQ(points_target.size(), points_reference.size());
  t_t_r.translation().normalize();

  ceres::Problem problem;
  problem.AddParameterBlock(t_t_r.data(), 7,
                            new ceres::ProductParameterization(new ceres::EigenQuaternionParameterization(),
                                                               new energy::problem::LocalParameterizationS2()));

  for (size_t i = 0; i < points_target.size(); ++i) {
    auto *cost = new ceres::AutoDiffCostFunction<cost_functors::SampsonErrorCost, 1, 7, 1>(
        new cost_functors::SampsonErrorCost(points_reference[i], points_target[i]));

    problem.AddResidualBlock(cost, new ceres::HuberLoss(threshold), t_t_r.data(), &focal);
  }

  if constexpr (!OPTIMIZE_FOCAL) {
    problem.SetParameterBlockConstant(&focal);
  }

  if constexpr (!OPTIMIZE_POSE) {
    problem.SetParameterBlockConstant(t_t_r.data());
  }

  ceres::Solver::Options options;
  options.num_threads = number_of_threads;
  options.linear_solver_type = ceres::DENSE_QR;
  options.logging_type = ceres::SILENT;
  options.max_num_iterations = 40;
  options.function_tolerance = 1e-10;
  options.parameter_tolerance = 1e-6;
  ceres::Solver::Summary summary;

  ceres::Solve(options, &problem, &summary);
  VLOG(1) << summary.FullReport();
}

template void refineSO3xS2<true, true>(double &, Sophus::SE3d &, const std::vector<Eigen::Vector2d> &,
                                       const std::vector<Eigen::Vector2d> &, double, const int);

template void refineSO3xS2<false, true>(double, Sophus::SE3d &, const std::vector<Eigen::Vector2d> &,
                                        const std::vector<Eigen::Vector2d> &, double, const int);

template void refineSO3xS2<true, false>(double &, Sophus::SE3d &, const std::vector<Eigen::Vector2d> &,
                                        const std::vector<Eigen::Vector2d> &, double, const int);

template void refineSO3xS2<false, false>(double, Sophus::SE3d &, const std::vector<Eigen::Vector2d> &,
                                         const std::vector<Eigen::Vector2d> &, double, const int);
}  // namespace dsopp::energy::problem
