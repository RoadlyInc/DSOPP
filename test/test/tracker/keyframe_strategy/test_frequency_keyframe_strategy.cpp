#include <gtest/gtest.h>
#include <sophus/se3.hpp>

#include "common/settings.hpp"
#include "sensor/synchronized_frame.hpp"
#include "track/active_odometry_track.hpp"
#include "track/frames/active_keyframe.hpp"
#include "tracker/keyframe_strategy/frequency_keyframe_strategy.hpp"

namespace dsopp {
namespace tracker {
namespace keyframe_strategy {
TEST(testFrequencyKeyframeStrategy, testFrequencyKeyframeStrategy) {
  using SE3 = energy::motion::SE3<Precision>;

  Precision frequency = 1;
  track::ActiveOdometryTrack<SE3> track;
  auto kf_strategy = std::make_unique<FrequencyKeyframeStrategy<SE3>>(frequency);
  for (size_t t = 0; t <= 5000; t += 500) {
    auto frame = std::make_unique<sensors::SynchronizedFrame>(1, time(std::chrono::milliseconds(t)));
    track::SLAMInternalTrackingFrame slam_internal_frame(0, frame->timestamp(), SE3(), SE3(), 1,
                                                         Eigen::Vector2<Precision>::Zero(), 0, 0);

    if (t % 1000 > 0) {
      ASSERT_FALSE(kf_strategy->needNewKeyframe(track, slam_internal_frame));
    } else {
      ASSERT_TRUE(kf_strategy->needNewKeyframe(track, slam_internal_frame));
      track.pushFrame(0, frame->timestamp(), SE3());
    }
  }
}
}  // namespace keyframe_strategy
}  // namespace tracker
}  // namespace dsopp
