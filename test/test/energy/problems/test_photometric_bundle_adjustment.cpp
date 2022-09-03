#include "energy/problems/photometric_bundle_adjustment/ceres_photometric_bundle_adjustment.hpp"
#include "energy/problems/photometric_bundle_adjustment/eigen_photometric_bundle_adjustment.hpp"
#include "energy/problems/photometric_bundle_adjustment/void_photometric_bundle_adjustment.hpp"
#include "energy/problems/pose_alignment/eigen_pose_alignment.hpp"

#include <numbers>
#include <thread>

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "common/settings.hpp"
#include "common/time/time.hpp"
#include "features/camera/ceres_grid.hpp"
#include "features/camera/pixel_data_frame.hpp"
#include "features/camera/pixel_map.hpp"
#include "features/camera/tracking_feature.hpp"
#include "sensor/synchronized_frame.hpp"
#include "sensors/camera/camera.hpp"
#include "test/tools/solver_test_data.hpp"
#include "test/tools/transformations_equality.hpp"
#include "track/active_odometry_track.hpp"
#include "track/connections/frame_connection.hpp"
#include "track/landmarks/active_tracking_landmark.hpp"

namespace dsopp {
namespace energy {
namespace problem {

using SE3 = motion::SE3<Precision>;

namespace {

template <class Scalar>
energy::problem::TrustRegionPhotometricBundleAdjustmentOptions<Scalar> trustRegionOptions(
    size_t iterations, Scalar initial_trust_region_radius, Scalar function_tolerance, Scalar parameter_tolerance,
    Scalar huber_loss) {
  return energy::problem::TrustRegionPhotometricBundleAdjustmentOptions<Scalar>(
      iterations, initial_trust_region_radius, function_tolerance, parameter_tolerance,
      Eigen::Vector2<Scalar>::Constant(static_cast<Scalar>(1e12)), static_cast<Scalar>(1e16), huber_loss);
}

template <typename PoseAligner, class Scalar>
void alignIteratively(test_tools::SolverTestData<SE3> &data, SE3 &t_first_last) {
  auto &odometry_track = data.track.odometryTrack();
  for (size_t keyframe_id = 1; keyframe_id < odometry_track.keyframes().size(); keyframe_id++) {
    const size_t kMaxIteration = 100;
    const Precision kInitialTrustRegionRadius = 1e2;
    const Precision kFunctionTolerance = 1e-8_p;
    const Precision kParameterTolerance = 1e-8_p;
    auto pose_aligner = PoseAligner(trustRegionOptions<Scalar>(kMaxIteration, kInitialTrustRegionRadius,
                                                               kFunctionTolerance, kParameterTolerance, 9));
    for (size_t i = 0; i < keyframe_id; i++)
      pose_aligner.pushFrame(*odometry_track.keyframes()[i], 0, *data.model, FrameParameterization::kFixed);
    pose_aligner.pushFrame(
        odometry_track.keyframes()[keyframe_id]->timestamp(), odometry_track.keyframes()[keyframe_id]->tWorldAgent(),
        odometry_track.keyframes()[keyframe_id]->pyramids(), {{data.sensor, data.camera->pyramidOfMasks()[0]}}, 1,
        Eigen::Vector2<Precision>::Zero(), 0, *data.model);
    pose_aligner.solve(1);
    odometry_track.keyframes()[keyframe_id]->setTWorldAgent(
        pose_aligner.getPose(odometry_track.keyframes()[keyframe_id]->timestamp()));

    t_first_last = pose_aligner.getPose(odometry_track.keyframes().front()->timestamp()).inverse() *
                   pose_aligner.getPose(odometry_track.keyframes()[keyframe_id]->timestamp());
  }
}
template <typename Solver, class Scalar>
void refine(test_tools::SolverTestData<SE3> &data, SE3 &t_first_last, Precision sigma_huber_loss) {
  const size_t kMaxIteration = 200;
  const Precision kInitialTrustRegionRadius = 1e5;
  const Precision kFunctionTolerance = 1e-8_p;
  const Precision kParameterTolerance = 1e-8_p;
  auto &odometry_track = data.track.odometryTrack();
  auto solver = Solver(trustRegionOptions<Scalar>(kMaxIteration, kInitialTrustRegionRadius, kFunctionTolerance,
                                                  kParameterTolerance, sigma_huber_loss));
  for (size_t keyframe_id = 0; keyframe_id < odometry_track.keyframes().size(); keyframe_id++) {
    solver.pushFrame(*odometry_track.keyframes()[keyframe_id], 0, *data.model,
                     keyframe_id == 0 ? FrameParameterization::kFixed : FrameParameterization::kFree);
  }
  solver.solve(1);
  for (size_t keyframe_id = 0; keyframe_id < odometry_track.keyframes().size(); keyframe_id++) {
    solver.updateFrame(*odometry_track.keyframes()[keyframe_id]);
  }
  t_first_last = solver.getPose(odometry_track.keyframes().front()->timestamp()).inverse() *
                 solver.getPose(odometry_track.keyframes().back()->timestamp());
}

template <int C>
void testCeresPhotometricBundleAdjustment() {
  std::vector<size_t> keyframe_ids({0, 5, 10, 15, 20});
  test_tools::SolverTestData<SE3> data(keyframe_ids, true);
  SE3 gt_first_last = data.gt_cam[0].inverse() * data.gt_cam[20];
  using PoseAligner = CeresPhotometricBundleAdjustment<SE3, test_tools::SolverTestData<SE3>::Model, Pattern::kSize,
                                                       features::PixelMap, true, false>;

  SE3 t_first_last_init;
  alignIteratively<PoseAligner, double>(data, t_first_last_init);
  SE3 t_first_last_refined;
  using Solver = CeresPhotometricBundleAdjustment<SE3, test_tools::SolverTestData<SE3>::Model, Pattern::kSize,
                                                  features::PixelMap, true, true, true, false, C>;

  refine<Solver, double>(data, t_first_last_refined, 9 * std::sqrt(static_cast<Precision>(C)));

  const Precision kMaxDistGT = 1e-2_p;
  const Precision kMaxAngleGT = std::numbers::pi_v<Precision> / 180;
  assertEqual(gt_first_last, t_first_last_refined, kMaxDistGT, kMaxAngleGT);
  // ensure refinement improved poses
  EXPECT_LE(angleError(gt_first_last, t_first_last_refined), angleError(gt_first_last, t_first_last_init));
}

template <int C>
void testEigenPhotometricBundleAdjustment() {
  std::vector<size_t> keyframe_ids({0, 5, 10, 15, 20});
  test_tools::SolverTestData<SE3> data(keyframe_ids, true);
  SE3 gt_first_last = data.gt_cam[0].inverse() * data.gt_cam[20];
  using PoseAligner = CeresPhotometricBundleAdjustment<SE3, test_tools::SolverTestData<SE3>::Model, Pattern::kSize,
                                                       features::PixelMap, true, false>;

  SE3 t_first_last_init;
  alignIteratively<PoseAligner, double>(data, t_first_last_init);

  SE3 t_first_last_refined;
  using Solver = EigenPhotometricBundleAdjustment<SE3, test_tools::SolverTestData<SE3>::Model, Pattern::kSize,
                                                  features::PixelMap, true, true, true, C>;

  refine<Solver, Precision>(data, t_first_last_refined, 9 * std::sqrt(static_cast<Precision>(C)));

  const Precision kMaxDistGT = 1e-2_p;
  const Precision kMaxAngleGT = std::numbers::pi_v<Precision> / 180;
  assertEqual(gt_first_last, t_first_last_refined, kMaxDistGT, kMaxAngleGT);
  EXPECT_LE(angleError(gt_first_last, t_first_last_refined), angleError(gt_first_last, t_first_last_init));
}

template <int C>
void testVoidPhotometricBundleAdjustment() {
  std::vector<size_t> keyframe_ids({0, 5, 10, 15, 20});
  test_tools::SolverTestData<SE3> data(keyframe_ids, false);
  SE3 gt_first_last = data.gt_cam[0].inverse() * data.gt_cam[20];

  SE3 t_first_last_refined;
  using Solver = VoidPhotometricBundleAdjustment<SE3, test_tools::SolverTestData<SE3>::Model, C>;

  auto &odometry_track = data.track.odometryTrack();
  auto solver = Solver();
  for (size_t keyframe_id = 0; keyframe_id < odometry_track.keyframes().size(); keyframe_id++) {
    solver.pushFrame(*odometry_track.keyframes()[keyframe_id], 0, *data.model,
                     keyframe_id == 0 ? FrameParameterization::kFixed : FrameParameterization::kFree);
  }
  solver.solve(1);
  for (size_t keyframe_id = 0; keyframe_id < odometry_track.keyframes().size(); keyframe_id++) {
    solver.updateFrame(*odometry_track.keyframes()[keyframe_id]);
  }
  t_first_last_refined = solver.getPose(odometry_track.keyframes().front()->timestamp()).inverse() *
                         solver.getPose(odometry_track.keyframes().back()->timestamp());

  EXPECT_LE(angleError(gt_first_last, t_first_last_refined), 1e-10);
}

template <int C>
void testEigenCeresCompareAfterOneIteration() {
  const Precision kEps = 1e-8_p;
  const size_t kMaxIteration = 1;
  const Precision kInitialTrustRegionRadius = 1e5;
  const Precision kFunctionTolerance = 1e-8_p;
  const Precision kParameterTolerance = 1e-8_p;

  std::vector<size_t> keyframe_ids({0, 5, 10, 15, 20});
  test_tools::SolverTestData<SE3> data(keyframe_ids, true, 10);
  auto &odometry_track = data.track.odometryTrack();

  using EigenSolver = EigenPhotometricBundleAdjustment<SE3, test_tools::SolverTestData<SE3>::Model, Pattern::kSize,
                                                       features::PixelMap, true, true, true, C>;
  auto eigen_solver =
      EigenSolver(trustRegionOptions<Precision>(kMaxIteration, kInitialTrustRegionRadius, kFunctionTolerance,
                                                kParameterTolerance, 20 * std::sqrt(static_cast<Precision>(C))),
                  true);
  for (size_t keyframe_id = 0; keyframe_id < odometry_track.keyframes().size(); keyframe_id++) {
    eigen_solver.pushFrame(*odometry_track.keyframes()[keyframe_id], 0, *data.model,
                           keyframe_id == 0 ? FrameParameterization::kFixed : FrameParameterization::kFree);
  }
  Precision eigen_energy = eigen_solver.solve(1);
  using CeresSolver = CeresPhotometricBundleAdjustment<SE3, test_tools::SolverTestData<SE3>::Model, Pattern::kSize,
                                                       features::PixelMap, true, true, true, false, C>;
  auto ceres_solver =
      CeresSolver(trustRegionOptions<double>(kMaxIteration, kInitialTrustRegionRadius, kFunctionTolerance,
                                             kParameterTolerance, 20 * std::sqrt(static_cast<Precision>(C))),
                  true);
  for (size_t keyframe_id = 0; keyframe_id < odometry_track.keyframes().size(); keyframe_id++) {
    ceres_solver.pushFrame(*odometry_track.keyframes()[keyframe_id], 0, *data.model,
                           keyframe_id == 0 ? FrameParameterization::kFixed : FrameParameterization::kFree);
  }
  Precision ceres_energy = ceres_solver.solve(1);
  EXPECT_LE(std::abs(eigen_energy - ceres_energy) / std::max(std::abs(eigen_energy), std::abs(ceres_energy)), 1e-4);
  for (const auto &keyframe : odometry_track.keyframes()) {
    // saving data from the eigen solver
    eigen_solver.updateFrame(*keyframe);
    auto eigen_pose = keyframe->tWorldAgent();
    std::vector<Eigen::Matrix<Precision, SE3::DoF, SE3::DoF>> eigen_covariances;
    for (const auto &target_keyframe : odometry_track.keyframes()) {
      if (target_keyframe->id() == keyframe->id()) {
        continue;
      }
      auto &connection = keyframe->getConnection(target_keyframe->keyframeId());
      eigen_covariances.push_back(connection.covariance());
    }
    std::vector<Precision> eigen_idepths;
    std::vector<Precision> eigen_idepths_variances;
    for (const auto &landmark : keyframe->activeLandmarks(data.sensor)) {
      eigen_idepths.push_back(landmark.idepth());
      eigen_idepths_variances.push_back(landmark.idepthVariance());
    }
    // saving data from the ceres solver
    ceres_solver.updateFrame(*keyframe);
    auto ceres_pose = keyframe->tWorldAgent();
    std::vector<Eigen::Matrix<Precision, SE3::DoF, SE3::DoF>> ceres_covariances;
    for (const auto &target_keyframe : odometry_track.keyframes()) {
      if (target_keyframe->id() == keyframe->id()) {
        continue;
      }
      auto &connection = keyframe->getConnection(target_keyframe->keyframeId());
      ceres_covariances.push_back(connection.covariance());
    }
    std::vector<Precision> ceres_idepths;
    std::vector<Precision> ceres_idepths_variances;
    for (const auto &landmark : keyframe->activeLandmarks(data.sensor)) {
      ceres_idepths.push_back(landmark.idepth());
      ceres_idepths_variances.push_back(landmark.idepthVariance());
    }
    // comparing
    assertEqual(eigen_pose, ceres_pose, kEps, kEps);
    for (size_t i = 0; i < eigen_idepths.size(); i++) {
      ASSERT_LE(std::abs(ceres_idepths_variances[i] - eigen_idepths_variances[i]),
                std::abs(eigen_idepths_variances[i]) * 1e-4 + 1e-4);
      ASSERT_LE(std::abs(ceres_idepths[i] - eigen_idepths[i]), std::abs(eigen_idepths[i]) * 1e-4 + 1e-4);
    }
    for (size_t i = 0; i < eigen_covariances.size(); i++) {
      EXPECT_LE((eigen_covariances[i] - ceres_covariances[i]).cwiseAbs().maxCoeff() /
                    ceres_covariances[i].cwiseAbs().maxCoeff(),
                1e-2);
      EXPECT_GE(eigen_covariances[i].norm(), 1e-10);
    }
  }
}
}  // namespace

TEST(photometric_bundle_adjustment, ceres_photometric_bundle_adjustment_one_channel) {
  testCeresPhotometricBundleAdjustment<1>();
}

TEST(photometric_bundle_adjustment, DISABLED_eigen_poses) {
  std::vector<size_t> keyframe_ids({0, 5, 10, 15, 20});
  test_tools::SolverTestData<SE3> data(keyframe_ids, false);
  SE3 gt_first_last = data.gt_cam[0].inverse() * data.gt_cam[20];
  const Precision kMaxDistGT = 1e-2_p;
  const Precision kMaxAngleGT = std::numbers::pi_v<Precision> / 180;
  // check that we do not spoil initial positions
  using Solver = EigenPhotometricBundleAdjustment<SE3, test_tools::SolverTestData<SE3>::Model, Pattern::kSize,
                                                  features::PixelMap, true, false>;
  SE3 t_first_last_refined;
  refine<Solver, Precision>(data, t_first_last_refined, 9);
  assertEqual(gt_first_last, t_first_last_refined, kMaxDistGT, kMaxAngleGT);
}

TEST(photometric_bundle_adjustment, eigen_photometric_bundle_adjustment_one_channel) {
  testEigenPhotometricBundleAdjustment<1>();
}

TEST(photometric_bundle_adjustment, void_photometric_bundle_adjustment_one_channel) {
  testVoidPhotometricBundleAdjustment<1>();
}

TEST(photometric_bundle_adjustment, eigen_ceres_compare_after_one_iteration_one_channel) {
  testEigenCeresCompareAfterOneIteration<1>();
}

}  // namespace problem
}  // namespace energy
}  // namespace dsopp
