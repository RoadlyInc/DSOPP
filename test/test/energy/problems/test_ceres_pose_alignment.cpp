#include <numbers>

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "energy/problems/photometric_bundle_adjustment/ceres_photometric_bundle_adjustment.hpp"
#include "energy/problems/pose_alignment/eigen_pose_alignment.hpp"

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
#include "track/active_track.hpp"
#include "track/landmarks/immature_tracking_landmark.hpp"

#include "tracker/create_depth_maps.hpp"

namespace dsopp {
namespace energy {
namespace problem {
template <class Scalar>
energy::problem::TrustRegionPhotometricBundleAdjustmentOptions<Scalar> trustRegionOptions(
    size_t iterations, Scalar initial_trust_region_radius, Scalar function_tolerance, Scalar parameter_tolerance,
    Scalar huber_loss, const int channels = 1) {
  return energy::problem::TrustRegionPhotometricBundleAdjustmentOptions<Scalar>(
      iterations, initial_trust_region_radius, function_tolerance, parameter_tolerance,
      Eigen::Vector2<Scalar>::Constant(static_cast<Scalar>(channels * 1e12)), static_cast<Scalar>(channels * 1e16),
      huber_loss * static_cast<Scalar>(std::sqrt(channels)));
}

class testPoseAlignment : public ::testing::Test {
 protected:
  const size_t kMaxIteration_ = 300;
  const Precision kInitialTrustRegionRadius_ = 1e2;
  const Precision kFunctionTolerance_ = 1e-8_p;
  const Precision kParameterTolerance_ = 1e-8_p;
  const Precision kMaxDistGT_ = 5e-2_p;
  const Precision kMaxAngleGT_ = 1 * std::numbers::pi_v<Precision> / 180;

  const Precision kHuberLossSigma_ = 20;
  using SE3 = motion::SE3<Precision>;
  using TestData = test_tools::SolverTestData<SE3>;

  void SetUp() override {
    std::vector<size_t> frame_ids = {0, 1};
    data = std::make_unique<TestData>(frame_ids);
    gt_cam0_cam1_gt = data->gt_cam[0].inverse() * data->gt_cam[1];
  }

  template <typename Solver, typename Model, int C = 1>
  motion::SE3<Precision> estimatePose(Solver& solver, const Model& model) {
    auto& odometry_track = data->track.odometryTrack();

    // create depth map
    std::map<size_t, std::vector<energy::problem::DepthMap>> sensor_depth_maps;
    std::vector<energy::problem::DepthMap> depth_maps;
    tracker::initDepthMaps(odometry_track.frames()[0]->pyramids().at(data->sensor), depth_maps);
    auto gt_depth_frame = data->gt_depth_maps.at(odometry_track.frames()[0]->id() - 10);
    for (const auto& landmark : odometry_track.frames()[0]->immatureLandmarks(data->sensor)) {
      Eigen::Vector2i coords = round(landmark.projection().array()).cast<int>();
      depth_maps[0].map(coords.x(), coords.y()).idepth +=
          1._p / (*gt_depth_frame)[static_cast<size_t>(coords(0))][static_cast<size_t>(coords(1))];
      depth_maps[0].map(coords.x(), coords.y()).weight += 1;
    }
    tracker::fillCoarseDepthMaps(depth_maps);
    tracker::dilateDepthMaps(depth_maps);
    sensor_depth_maps[data->sensor] = depth_maps;

    if constexpr (C == 1) {
      solver.pushFrame(odometry_track.frames()[0]->timestamp(), odometry_track.frames()[0]->tWorldAgent(),
                       odometry_track.frames()[0]->pyramids(), {{data->sensor, data->camera->pyramidOfMasks()[0]}},
                       sensor_depth_maps, 1, Eigen::Vector2<Precision>::Zero(), 0, model,
                       FrameParameterization::kFixed);
      solver.pushFrame(odometry_track.frames()[1]->timestamp(), SE3(), odometry_track.frames()[1]->pyramids(),
                       {{data->sensor, data->camera->pyramidOfMasks()[0]}}, 1, Eigen::Vector2<Precision>::Zero(), 0,
                       model);

    } else {
      odometry_track.getActiveKeyframe(1).setTWorldAgent(energy::motion::SE3<Precision>());
      solver.pushFrame(*odometry_track.keyframes()[0], 0, *data->model, FrameParameterization::kFixed);
      solver.pushFrame(*odometry_track.keyframes()[1], 0, *data->model, FrameParameterization::kFree);
    }

    solver.solve(4);

    return solver.getPose(odometry_track.frames()[0]->timestamp()).inverse() *
           solver.getPose(odometry_track.frames()[1]->timestamp());
  }

  std::unique_ptr<TestData> data;
  SE3 gt_cam0_cam1_gt;
};

TEST_F(testPoseAlignment, ceresEightPixel) {
  const int kPatternSize = 8;
  auto solver =
      CeresPhotometricBundleAdjustment<SE3, TestData::Model, kPatternSize, features::PixelMap, true, false, false>(
          trustRegionOptions<double>(kMaxIteration_, kInitialTrustRegionRadius_, kFunctionTolerance_,
                                     kParameterTolerance_, kHuberLossSigma_));

  auto gt_cam0_cam1_estimated = estimatePose(solver, *data->model);
  assertEqual(gt_cam0_cam1_gt, gt_cam0_cam1_estimated, kMaxDistGT_, kMaxAngleGT_);
}

TEST_F(testPoseAlignment, ceresOnePixel) {
  const int kPatternSize = 1;
  auto solver =
      CeresPhotometricBundleAdjustment<SE3, TestData::Model, kPatternSize, features::PixelMap, true, false, false>(
          trustRegionOptions<double>(kMaxIteration_, kInitialTrustRegionRadius_, kFunctionTolerance_,
                                     kParameterTolerance_, kHuberLossSigma_));

  auto gt_cam0_cam1_estimated = estimatePose(solver, *data->model);
  assertEqual(gt_cam0_cam1_gt, gt_cam0_cam1_estimated, kMaxDistGT_, kMaxAngleGT_);
}

TEST_F(testPoseAlignment, eigenEightPixel) {
  const int kPatternSize = 8;
  auto solver = EigenPoseAlignment<SE3, test_tools::SolverTestData<SE3>::Model, kPatternSize, features::CeresGrid>(
      trustRegionOptions<Precision>(kMaxIteration_, kInitialTrustRegionRadius_, kFunctionTolerance_,
                                    kParameterTolerance_, kHuberLossSigma_));

  auto gt_cam0_cam1_estimated = estimatePose(solver, *data->model);
  assertEqual(gt_cam0_cam1_gt, gt_cam0_cam1_estimated, kMaxDistGT_, kMaxAngleGT_);
}

TEST_F(testPoseAlignment, eigenOnePixel) {
  const int kPatternSize = 1;
  auto solver = EigenPoseAlignment<SE3, test_tools::SolverTestData<SE3>::Model, kPatternSize, features::CeresGrid>(
      trustRegionOptions<Precision>(kMaxIteration_, kInitialTrustRegionRadius_, kFunctionTolerance_,
                                    kParameterTolerance_, kHuberLossSigma_));

  auto gt_cam0_cam1_estimated = estimatePose(solver, *data->model);
  assertEqual(gt_cam0_cam1_gt, gt_cam0_cam1_estimated, kMaxDistGT_, kMaxAngleGT_);
}
}  // namespace problem
}  // namespace energy
}  // namespace dsopp
