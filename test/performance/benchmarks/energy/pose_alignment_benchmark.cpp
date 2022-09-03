#include "energy/problems/photometric_bundle_adjustment/ceres_photometric_bundle_adjustment.hpp"
#include "energy/problems/pose_alignment/eigen_pose_alignment.hpp"

#include <benchmark/benchmark.h>

#include "common/settings.hpp"
#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"
#include "features/camera/pixel_map.hpp"
#include "sensor/synchronized_frame.hpp"
#include "test/tools/solver_test_data.hpp"
#include "track/active_odometry_track.hpp"

using namespace dsopp;
using SE3 = energy::motion::SE3<Precision>;
static void CeresPoseAligment(benchmark::State& state) {
  test_tools::SolverTestData<SE3> data({0, 1});
  const Precision kInitialTrustRegionRadius = 1e2;

  for (auto _ : state) {
    auto ceres_solver = energy::problem::CeresPhotometricBundleAdjustment<SE3, test_tools::SolverTestData<SE3>::Model,
                                                                          1, features::PixelMap, true, false, false>(
        energy::problem::TrustRegionPhotometricBundleAdjustmentOptions<double>(
            1, kInitialTrustRegionRadius, 0, 0, Eigen::Vector2<double>::Constant(static_cast<double>(1e12)),
            static_cast<double>(1e16), 5));

    auto& odometry_track = data.track.odometryTrack();
    ceres_solver.pushFrame(*odometry_track.keyframes()[0], 0, *data.model,
                           energy::problem::FrameParameterization::kFixed);
    ceres_solver.pushFrame(odometry_track.keyframes()[1]->timestamp(), SE3(), odometry_track.keyframes()[1]->pyramids(),
                           {{data.sensor, data.camera->pyramidOfMasks()[0]}}, 1, Eigen::Vector2<Precision>::Zero(), 0,
                           *data.model);
    ceres_solver.solve(1);
  }
}

static void EigenPoseAligment(benchmark::State& state) {
  test_tools::SolverTestData<SE3> data({0, 1});
  const Precision kInitialTrustRegionRadius = 1e2;
  for (auto _ : state) {
    auto eigen_solver =
        energy::problem::EigenPoseAlignment<SE3, test_tools::SolverTestData<SE3>::Model, 1, features::PixelMap>(
            energy::problem::TrustRegionPhotometricBundleAdjustmentOptions<Precision>(
                1, kInitialTrustRegionRadius, 0, 0, Eigen::Vector2<Precision>::Constant(static_cast<Precision>(1e12)),
                static_cast<Precision>(1e16), 5));

    auto& odometry_track = data.track.odometryTrack();
    eigen_solver.pushFrame(*odometry_track.keyframes()[0], 0, *data.model,
                           energy::problem::FrameParameterization::kFixed);
    eigen_solver.pushFrame(odometry_track.keyframes()[1]->timestamp(), SE3(), odometry_track.keyframes()[1]->pyramids(),
                           {{data.sensor, data.camera->pyramidOfMasks()[0]}}, 1, Eigen::Vector2<Precision>::Zero(), 0,
                           *data.model);
    eigen_solver.solve(1);
  }
}

BENCHMARK(CeresPoseAligment)->Unit(benchmark::kMillisecond);
BENCHMARK(EigenPoseAligment)->Unit(benchmark::kMillisecond);
