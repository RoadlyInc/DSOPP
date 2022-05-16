#include "energy/problems/photometric_bundle_adjustment/bundle_adjustment_photometric_evaluation_callback.hpp"
#include "energy/problems/photometric_bundle_adjustment/ceres_photometric_bundle_adjustment.hpp"
#include "energy/problems/pose_alignment/eigen_pose_alignment.hpp"

#include "common/time/time.hpp"
#include "features/camera/ceres_grid.hpp"
#include "features/camera/pixel_data_frame.hpp"
#include "features/camera/pixel_map.hpp"
#include "features/camera/tracking_feature.hpp"
#include "sensor/synchronized_frame.hpp"
#include "sensors/camera/camera.hpp"
#include "test/tools/solver_test_data.hpp"
#include "test/tools/transformations_equality.hpp"
#include "track/active_track.hpp"

#include <gtest/gtest.h>
#include <Eigen/Dense>

namespace dsopp {
namespace energy {
namespace problem {

TEST(testCameraOptimization, testCameraOptimization) {
  using SE3 = motion::SE3<Precision>;
  using TestData = test_tools::SolverTestData<SE3, features::CeresGrid, false>;

  const size_t kMaxIteration = 500;
  const Precision kInitialTrustRegionRadius = 1e2;
  const Precision kFunctionTolerance = 1e-8_p;
  const Precision kParameterTolerance = 1e-8_p;
  const Precision kHuberLossSigma = 20;

  const Precision kPerturbation = 0.05_p;
  const Precision kError = 0.1_p * kPerturbation;

  std::vector<size_t> frame_ids;
  for (size_t i = 0; i < 20; i++) {
    frame_ids.push_back(250 + i * 5);
  }
  TestData data{frame_ids, false, 50};

  Eigen::Vector2<Precision> focal_lengths_gt = data.model->focal_lengths();
  Eigen::Vector2<Precision> principal_point_gt = data.model->principal_point();

  Eigen::Vector2<Precision> focal_lengths = focal_lengths_gt * (1 - kPerturbation);
  Eigen::Vector2<Precision> principal_point = principal_point_gt;

  auto calibration = std::make_unique<TestData::Calibration>(
      data.camera->calibration().image_size(),
      Eigen::Vector4<Precision>(focal_lengths(0), focal_lengths(1), principal_point(0), principal_point(1)),
      data.camera->calibration().type());

  const int kPatternSize = 8;
  auto ceres_solver = CeresPhotometricBundleAdjustment<SE3, TestData::Model, kPatternSize, features::PixelMap, false,
                                                       false, false, true>(
      energy::problem::TrustRegionPhotometricBundleAdjustmentOptions<double>(
          kMaxIteration, kInitialTrustRegionRadius, kFunctionTolerance, kParameterTolerance,
          Eigen::Vector2<double>::Constant(1e12), 1e16, kHuberLossSigma),
      false, false, 0);
  auto& odometry_track = data.track.odometryTrack();

  for (size_t keyframe_id = 0; keyframe_id < odometry_track.keyframes().size(); keyframe_id++) {
    ceres_solver.pushFrame(*odometry_track.keyframes()[keyframe_id], 0, *data.model,
                           keyframe_id == 0 ? FrameParameterization::kFixed : FrameParameterization::kFree);
  }
  ceres_solver.solve(4);

  auto optimized_model = ceres_solver.optimizedCameraModel(0);

  CHECK_LE((optimized_model.focal_lengths() - focal_lengths_gt).norm() / focal_lengths_gt.norm(), kError);
  CHECK_LE((optimized_model.principal_point() - principal_point_gt).norm() / principal_point_gt.norm(), kError);
}
}  // namespace problem
}  // namespace energy
}  // namespace dsopp
