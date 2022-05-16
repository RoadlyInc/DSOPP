#include "energy/problems/photometric_bundle_adjustment/ceres_photometric_bundle_adjustment.hpp"

#include <gtest/gtest.h>

#include "common/settings.hpp"
#include "features/camera/pixel_map.hpp"
#include "test/tools/solver_test_data.hpp"
#include "track/connections/connections_container.hpp"
#include "track/connections/frame_connection.hpp"
#include "track/landmarks/active_tracking_landmark.hpp"

namespace dsopp {
namespace energy {
namespace problem {
using SE3 = motion::SE3<Precision>;
void addConnectionClique(auto &odometry_track, size_t start, size_t end) {
  size_t sensor = 0;
  for (size_t reference_keyframe_id = start; reference_keyframe_id < end; reference_keyframe_id++) {
    for (size_t target_keyframe_id = start; target_keyframe_id < end; target_keyframe_id++) {
      auto reference_frame = odometry_track.keyframes()[reference_keyframe_id];
      auto target_frame = odometry_track.keyframes()[target_keyframe_id];
      if (reference_frame->id() >= target_frame->id()) continue;
      if (odometry_track.connections().exists(reference_frame->id(), target_frame->id())) continue;
      auto connection = std::make_unique<track::FrameConnection<SE3>>(reference_frame->id(), target_frame->id());
      connection->addSensorConnection(sensor, sensor, reference_frame->activeLandmarks(sensor).size(),
                                      target_frame->activeLandmarks(sensor).size());
      reference_frame->addConnection(target_frame->id(), connection.get());
      target_frame->addConnection(reference_frame->id(), connection.get());
      odometry_track.connections().add(std::move(connection));
    }
  }
}
TEST(incremental_solver, incremental_solver) {
  test_tools::SolverTestData<SE3> data({0, 10, 20, 30, 40, 50, 60, 70, 80, 90}, false, 50, false);

  using Solver = CeresPhotometricBundleAdjustment<SE3, test_tools::SolverTestData<SE3>::Model, Pattern::kSize,
                                                  features::PixelMap, true, false>;
  const size_t kMaxIteration = 200;
  const Precision kInitialTrustRegionRadius = 1e5;
  const Precision kFunctionTolerance = 1e-8_p;
  const Precision kParameterTolerance = 1e-8_p;
  auto &odometry_track = data.track.odometryTrack();
  auto solver = Solver(energy::problem::TrustRegionPhotometricBundleAdjustmentOptions<double>(
      kMaxIteration, kInitialTrustRegionRadius, kFunctionTolerance, kParameterTolerance,
      Eigen::Vector2<double>::Constant(1e12), 1e16, 9));

  // add connections between first 3 keyframes
  addConnectionClique(odometry_track, 0, 3);

  // add first 5 frames to solver
  for (size_t keyframe_id = 0; keyframe_id < 3; keyframe_id++) {
    solver.pushFrame(*odometry_track.keyframes()[keyframe_id], 0, *data.model,
                     keyframe_id == 0 ? FrameParameterization::kFixed : FrameParameterization::kFree);
  }
  solver.solve(1);

  // marginalize first 2 frames
  for (size_t keyframe_id = 0; keyframe_id < 2; keyframe_id++) {
    auto &frame = odometry_track.getActiveKeyframe(0);
    odometry_track.marginalizeFrame(0);
    solver.updateLocalFrame(frame);
  }
  solver.solve(1);

  // add 2 new frames
  addConnectionClique(odometry_track, 3, 5);
  for (size_t keyframe_id = 3; keyframe_id < 5; keyframe_id++) {
    solver.pushFrame(*odometry_track.keyframes()[keyframe_id], 0, *data.model, FrameParameterization::kFree);
  }
  solver.solve(1);

  // marginalize 3rd frame to test behaviour on frame deletetion
  {
    auto &frame = odometry_track.getActiveKeyframe(0);
    odometry_track.marginalizeFrame(0);
    solver.updateLocalFrame(frame);
  }
  solver.solve(1);
  // add the rest
  addConnectionClique(odometry_track, 4, 10);
  for (size_t keyframe_id = 5; keyframe_id < 10; keyframe_id++) {
    solver.pushFrame(*odometry_track.keyframes()[keyframe_id], 0, *data.model, FrameParameterization::kFree);
  }
  solver.solve(1);
  // try to delete from the center
  for (size_t keyframe_id = 0; keyframe_id < 2; keyframe_id++) {
    auto &frame = odometry_track.getActiveKeyframe(1);
    odometry_track.marginalizeFrame(1);
    solver.updateLocalFrame(frame);
  }

  // marginalized almost everything

  while (odometry_track.activeFrames().size() > 1) {
    auto &frame = odometry_track.getActiveKeyframe(0);
    odometry_track.marginalizeFrame(0);
    solver.updateLocalFrame(frame);
  }
  solver.solve(1);
}
}  // namespace problem
}  // namespace energy
}  // namespace dsopp
