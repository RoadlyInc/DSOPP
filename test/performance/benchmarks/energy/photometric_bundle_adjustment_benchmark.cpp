#include "energy/problems/photometric_bundle_adjustment/ceres_photometric_bundle_adjustment.hpp"
#include "energy/problems/photometric_bundle_adjustment/eigen_photometric_bundle_adjustment.hpp"

#include "energy/levenberg_marquardt_algorithm/levenberg_marquardt_algorithm.hpp"
#include "energy/problems/photometric_bundle_adjustment/eigen_photometric_bundle_adjustment_problem.hpp"
#include "energy/problems/photometric_bundle_adjustment/evaluate_jacobians.hpp"
#include "energy/problems/photometric_bundle_adjustment/first_estimate_jacobians.hpp"
#include "energy/problems/photometric_bundle_adjustment/hessian_block_evaluation.hpp"

#include <benchmark/benchmark.h>

#include "common/settings.hpp"
#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"
#include "energy/normal_linear_system.hpp"
#include "features/camera/pixel_map.hpp"
#include "sensor/synchronized_frame.hpp"
#include "test/tools/solver_test_data.hpp"
#include "track/active_odometry_track.hpp"

using namespace dsopp;
using namespace dsopp::energy;
using namespace dsopp::energy::problem;
using SE3 = energy::motion::SE3<Precision>;

static void CeresPhotometricBundleAdjustmentBenchmark(benchmark::State& state) {
  test_tools::SolverTestData<SE3> data({0, 5, 10, 15, 20, 25, 30, 35, 40}, true);
  const Precision kInitialTrustRegionRadius = 1e5;
  auto& odometry_track = data.track.odometryTrack();

  for (auto _ : state) {
    auto ceres_solver =
        energy::problem::CeresPhotometricBundleAdjustment<SE3, test_tools::SolverTestData<SE3>::Model, Pattern::kSize,
                                                          features::PixelMap, true, true, true>(
            energy::problem::TrustRegionPhotometricBundleAdjustmentOptions<double>(
                50, kInitialTrustRegionRadius, 0, 0, Eigen::Vector2<double>::Constant(1e12), 1e16, 5));
    for (size_t i = 0; i < odometry_track.keyframes().size(); i++) {
      ceres_solver.pushFrame(
          *odometry_track.keyframes()[i], 0, *data.model,
          i == 0 ? energy::problem::FrameParameterization::kFixed : energy::problem::FrameParameterization::kFree);
    }
    ceres_solver.solve(4);
  }
}

static void EigenPhotometricBundleAdjustmentBenchmark(benchmark::State& state) {
  test_tools::SolverTestData<SE3> data({0, 5, 10, 15, 20, 25, 30, 35, 40}, true);
  const Precision kInitialTrustRegionRadius = 1e5;
  for (auto _ : state) {
    auto eigen_solver =
        energy::problem::EigenPhotometricBundleAdjustment<SE3, test_tools::SolverTestData<SE3>::Model, Pattern::kSize,
                                                          features::PixelMap, true, true, true>(
            energy::problem::TrustRegionPhotometricBundleAdjustmentOptions<Precision>(
                50, kInitialTrustRegionRadius, 0, 0, Eigen::Vector2<Precision>::Constant(static_cast<Precision>(1e12)),
                static_cast<Precision>(1e16), 5));
    auto& odometry_track = data.track.odometryTrack();
    for (size_t i = 0; i < odometry_track.keyframes().size(); i++) {
      eigen_solver.pushFrame(
          *odometry_track.keyframes()[i], 0, *data.model,
          i == 0 ? energy::problem::FrameParameterization::kFixed : energy::problem::FrameParameterization::kFree);
    }
    eigen_solver.solve(1);
  }
}

class LinearSystemBenchmarkData {
 public:
  using SE3 = energy::motion::SE3<Precision>;
  static const size_t kBlockSize = SE3::DoF + 2;
  static const int kPatternSize = 8;
  static constexpr Precision kHuberSigma = 9.0;
  static const int C = 1;
  test_tools::SolverTestData<SE3> test_data{{0, 5, 10, 15, 20, 25, 30, 35, 40}, true};
  const size_t kNumFrames = test_data.track.odometryTrack().keyframes().size();
  LinearSystemBenchmarkData() {
    auto solver =
        energy::problem::EigenPhotometricBundleAdjustment<SE3, test_tools::SolverTestData<SE3>::Model, Pattern::kSize,
                                                          features::PixelMap, true, true, true>(
            energy::problem::TrustRegionPhotometricBundleAdjustmentOptions<Precision>(
                1, 1e5, 0, 0, Eigen::Vector2<Precision>::Constant(static_cast<Precision>(1e12)),
                static_cast<Precision>(1e16), 5));
    auto& odometry_track = test_data.track.odometryTrack();
    for (size_t i = 0; i < odometry_track.keyframes().size(); i++) {
      solver.pushFrame(
          *odometry_track.keyframes()[i], 0, *test_data.model,
          i == 0 ? energy::problem::FrameParameterization::kFixed : energy::problem::FrameParameterization::kFree);
    }
    frames = std::move(solver.frames_);
    firstEstimateJacobians_<Precision, SE3>(frames);
    evaluateJacobians<Precision, SE3, typename test_tools::SolverTestData<SE3>::Model, Pattern::kSize,
                      features::PixelMap, C, false, true, true, true, true>(frames, kHuberSigma);
  }

  std::deque<std::unique_ptr<
      LocalFrame<Precision, SE3, test_tools::SolverTestData<SE3>::Model, kPatternSize, features::PixelMap, C>>>
      frames;
};
static void EigenPhotometricBundleAdjustmentLinearSystemEvaluateJacobiansBenchmark(benchmark::State& state) {
  LinearSystemBenchmarkData data;

  for (auto _ : state) {
    evaluateJacobians<Precision, SE3, typename test_tools::SolverTestData<SE3>::Model, Pattern::kSize,
                      features::PixelMap, data.C, false, true, true, true, true>(data.frames, data.kHuberSigma);
  }
}

static void EigenPhotometricBundleAdjustmentLinearSystemEvaluateResidualsBenchmark(benchmark::State& state) {
  LinearSystemBenchmarkData data;

  for (auto _ : state) {
    evaluateJacobians<Precision, SE3, typename test_tools::SolverTestData<SE3>::Model, Pattern::kSize,
                      features::PixelMap, data.C, false, true, false, true, true>(data.frames, data.kHuberSigma);
  }
}

static void EigenPhotometricBundleAdjustmentLinearSystemPosePoseBenchmark(benchmark::State& state) {
  LinearSystemBenchmarkData data;

  NormalLinearSystem system_pose(static_cast<int>(data.kNumFrames * data.kBlockSize));
  for (auto _ : state) {
    system_pose.setZero();
    evaluateLinearSystemPosePose(data.frames, data.test_data.sensor, system_pose);
  }
}

static void EigenPhotometricBundleAdjustmentLinearSystemSchurBenchmark(benchmark::State& state) {
  LinearSystemBenchmarkData data;

  NormalLinearSystem system(static_cast<int>(data.kNumFrames * data.kBlockSize));
  for (auto _ : state) {
    system.setZero();
    evaluateLinearSystemPoseDepthSchurComplement(data.frames, data.test_data.sensor, system);
  }
}
static void EigenPhotometricBundleAdjustmentLinearCalculateIdepthsBenchmark(benchmark::State& state) {
  LinearSystemBenchmarkData data;
  NormalLinearSystem system(static_cast<int>(data.kNumFrames * data.kBlockSize));
  system.setZero();

  Eigen::VectorX<Precision> step(static_cast<int>(data.kNumFrames * data.kBlockSize));
  evaluateLinearSystemPoseDepthSchurComplement(data.frames, data.test_data.sensor, system);
  step.setZero();
  for (auto _ : state) {
    calculateIdepths(data.frames, data.test_data.sensor, step, 1e-5_p);
  }
}
static void EigenPhotometricBundleAdjustmentLinearize(benchmark::State& state) {
  LinearSystemBenchmarkData data;
  /** marginalized part of the problem */
  NormalLinearSystem system_marginalized{static_cast<int>(data.kBlockSize * data.frames.size())};
  /** marginalized part of the problem */
  system_marginalized.setZero();
  PhotometricBundleAdjustmentProblem<SE3, typename test_tools::SolverTestData<SE3>::Model, Pattern::kSize,
                                     features::PixelMap, true, true, true, data.C>
      problem(data.frames, 0, data.kHuberSigma, system_marginalized, 0, Eigen::Vector2<Precision>::Constant(1e12_p),
              1e16_p);
  for (auto _ : state) {
    problem.linearize();
  }
}
static void EigenPhotometricBundleAdjustmentCalculateStep(benchmark::State& state) {
  LinearSystemBenchmarkData data;
  /** marginalized part of the problem */
  NormalLinearSystem system_marginalized{static_cast<int>(data.kBlockSize * data.frames.size())};
  /** marginalized part of the problem */
  system_marginalized.setZero();
  PhotometricBundleAdjustmentProblem<SE3, typename test_tools::SolverTestData<SE3>::Model, Pattern::kSize,
                                     features::PixelMap, true, true, true, data.C>
      problem(data.frames, 0, data.kHuberSigma, system_marginalized, 0, Eigen::Vector2<Precision>::Constant(1e12_p),
              1e16_p);
  problem.linearize();
  for (auto _ : state) {
    problem.calculateStep(1e-5_p);
  }
}
static void EigenPhotometricBundleAdjustmentCalculateEnergy(benchmark::State& state) {
  LinearSystemBenchmarkData data;
  /** marginalized part of the problem */
  NormalLinearSystem system_marginalized{static_cast<int>(data.kBlockSize * data.frames.size())};
  /** marginalized part of the problem */
  system_marginalized.setZero();
  PhotometricBundleAdjustmentProblem<SE3, typename test_tools::SolverTestData<SE3>::Model, Pattern::kSize,
                                     features::PixelMap, true, true, true, data.C>
      problem(data.frames, 0, data.kHuberSigma, system_marginalized, 0, Eigen::Vector2<Precision>::Constant(1e12_p),
              1e16_p);
  for (auto _ : state) {
    problem.calculateEnergy();
  }
}
static void EigenPhotometricBundleAdjustmentAcceptStep(benchmark::State& state) {
  LinearSystemBenchmarkData data;
  /** marginalized part of the problem */
  NormalLinearSystem system_marginalized{static_cast<int>(data.kBlockSize * data.frames.size())};
  /** marginalized part of the problem */
  system_marginalized.setZero();
  PhotometricBundleAdjustmentProblem<SE3, typename test_tools::SolverTestData<SE3>::Model, Pattern::kSize,
                                     features::PixelMap, true, true, true, data.C>
      problem(data.frames, 0, data.kHuberSigma, system_marginalized, 0, Eigen::Vector2<Precision>::Constant(1e12_p),
              1e16_p);
  for (auto _ : state) {
    problem.acceptStep();
  }
}
static void EigenPhotometricBundleAdjustmentRejectStep(benchmark::State& state) {
  LinearSystemBenchmarkData data;
  /** marginalized part of the problem */
  NormalLinearSystem system_marginalized{static_cast<int>(data.kBlockSize * data.frames.size())};
  /** marginalized part of the problem */
  system_marginalized.setZero();
  PhotometricBundleAdjustmentProblem<SE3, typename test_tools::SolverTestData<SE3>::Model, Pattern::kSize,
                                     features::PixelMap, true, true, true, data.C>
      problem(data.frames, 0, data.kHuberSigma, system_marginalized, 0, Eigen::Vector2<Precision>::Constant(1e12_p),
              1e16_p);
  for (auto _ : state) {
    problem.rejectStep();
  }
}

static void EigenPhotometricBundleAdjustmentLevenbergMarquardtBenchmark(benchmark::State& state) {
  LinearSystemBenchmarkData data;
  /** marginalized part of the problem */
  NormalLinearSystem system_marginalized{static_cast<int>(data.kBlockSize * data.frames.size())};
  /** marginalized part of the problem */
  system_marginalized.setZero();
  PhotometricBundleAdjustmentProblem<SE3, typename test_tools::SolverTestData<SE3>::Model, Pattern::kSize,
                                     features::PixelMap, true, true, true, data.C>
      problem(data.frames, 0, data.kHuberSigma, system_marginalized, 0, Eigen::Vector2<Precision>::Constant(1e12_p),
              1e16_p);
  for (auto _ : state) {
    energy::levenberg_marquardt_algorithm::Options options;
    options.initial_levenberg_marquardt_regularizer = 1._p / 1e5_p;
    options.function_tolerance = 0;
    options.parameter_tolerance = 0;
    options.max_num_iterations = 1;
    options.levenberg_marquardt_regularizer_decrease_on_accept = 2.;
    options.levenberg_marquardt_regularizer_increase_on_reject = 10.;
    energy::levenberg_marquardt_algorithm::solve(problem, options);
  }
}

static void EigenPhotometricBundleAdjustmentSolveSystemBenchmark(benchmark::State& state) {
  LinearSystemBenchmarkData data;

  NormalLinearSystem system(static_cast<int>(data.kNumFrames * data.kBlockSize));
  system.setZero();
  evaluateLinearSystemPosePose(data.frames, data.test_data.sensor, system);
  for (auto _ : state) {
    system.solve();
  }
}

BENCHMARK(EigenPhotometricBundleAdjustmentLinearSystemEvaluateJacobiansBenchmark)->Unit(benchmark::kMillisecond);
BENCHMARK(EigenPhotometricBundleAdjustmentLinearSystemEvaluateResidualsBenchmark)->Unit(benchmark::kMillisecond);
BENCHMARK(EigenPhotometricBundleAdjustmentLinearSystemPosePoseBenchmark)->Unit(benchmark::kMillisecond);
BENCHMARK(EigenPhotometricBundleAdjustmentLinearSystemSchurBenchmark)->Unit(benchmark::kMillisecond);
BENCHMARK(EigenPhotometricBundleAdjustmentLinearCalculateIdepthsBenchmark)->Unit(benchmark::kMillisecond);

BENCHMARK(EigenPhotometricBundleAdjustmentLinearize)->Unit(benchmark::kMillisecond);
BENCHMARK(EigenPhotometricBundleAdjustmentCalculateStep)->Unit(benchmark::kMillisecond);
BENCHMARK(EigenPhotometricBundleAdjustmentCalculateEnergy)->Unit(benchmark::kMillisecond);
BENCHMARK(EigenPhotometricBundleAdjustmentAcceptStep)->Unit(benchmark::kMillisecond);
BENCHMARK(EigenPhotometricBundleAdjustmentRejectStep)->Unit(benchmark::kMillisecond);
BENCHMARK(EigenPhotometricBundleAdjustmentLevenbergMarquardtBenchmark)->Unit(benchmark::kMillisecond);
BENCHMARK(EigenPhotometricBundleAdjustmentSolveSystemBenchmark)->Unit(benchmark::kMillisecond);

BENCHMARK(CeresPhotometricBundleAdjustmentBenchmark)->Unit(benchmark::kMillisecond);
BENCHMARK(EigenPhotometricBundleAdjustmentBenchmark)->Unit(benchmark::kMillisecond);
