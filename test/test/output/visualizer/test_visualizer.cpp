#include <memory>

#include <gtest/gtest.h>

#include "agent/agent.hpp"
#include "common/settings.hpp"
#include "common/time/time.hpp"
#include "output_interfaces/track_output_interface.hpp"
#include "sensors/sensors.hpp"
#include "track/active_odometry_track.hpp"
#include "track/active_track.hpp"
#include "track/frames/active_keyframe.hpp"
#include "visualizer/visualizer.hpp"

namespace dsopp {
namespace output {

TEST(testVisualizer, testVisualizer) {
  using SE3 = energy::motion::SE3<Precision>;

  auto visualizer = std::make_unique<dsopp::output::Visualizer>(1920, 1080);
  auto *visualizer_oi1 = visualizer->createTrackOutputInterface<dsopp::track::ActiveOdometryTrack, SE3>();
  auto *visualizer_oi2 = visualizer->createTrackOutputInterface<dsopp::track::ActiveOdometryTrack, SE3>();
  Sophus::SE3<Precision> g1;
  Sophus::SE3<Precision> g2;
  g2.translation() = Eigen::Vector3<Precision>(0, 0, 3);
  size_t frame_id = 0;
  auto track1 = std::make_shared<track::ActiveTrack<SE3>>();
  auto &odometry_track1 = track1->odometryTrack();
  odometry_track1.pushFrame(frame_id++, time(std::chrono::milliseconds(0)), g1);
  odometry_track1.pushFrame(frame_id++, time(std::chrono::milliseconds(1)), g2);

  g1.translation() = Eigen::Vector3<Precision>(2, 0, 0);
  g2.translation() = Eigen::Vector3<Precision>(2, 0, 3);
  auto track2 = std::make_shared<track::ActiveTrack<SE3>>();
  auto &odometry_track2 = track1->odometryTrack();
  odometry_track2.pushFrame(frame_id++, time(std::chrono::milliseconds(0)), g1);
  odometry_track2.pushFrame(frame_id++, time(std::chrono::milliseconds(1)), g2);

  visualizer_oi1->notify(*track1);
  visualizer_oi2->notify(*track2);
}

}  // namespace output
}  // namespace dsopp
