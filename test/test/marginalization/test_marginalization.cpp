#include "marginalization/maximum_size_frame_marginalization_strategy.hpp"

#include "common/settings.hpp"
#include "common/time/time.hpp"
#include "track/active_odometry_track.hpp"
#include "track/frames/active_keyframe.hpp"

#include <gtest/gtest.h>
#include <memory>

namespace dsopp {
namespace marginalization {
TEST(max_size_marginalization, max_size_marginalization) {
  using SE3 = energy::motion::SE3<Precision>;

  const size_t kMaxFrames = 8;
  const size_t kFramesToCheck = 30;
  std::unique_ptr<FrameMarginalizationStrategy<SE3>> marginalizer =
      std::make_unique<MaximumSizeFrameMarginalizationStrategy<SE3>>(kMaxFrames);

  size_t expected_active_frames = 0;
  track::ActiveOdometryTrack<SE3> track;
  for (size_t i = 0; i < kFramesToCheck; i++) {
    EXPECT_EQ(track.activeFrames().size(), expected_active_frames);
    track.pushFrame(i, time(std::chrono::milliseconds(0)), Sophus::SE3<Precision>());
    expected_active_frames++;
    EXPECT_EQ(track.activeFrames().size(), expected_active_frames);
    marginalizer->marginalize(track);
    expected_active_frames = std::min(expected_active_frames, kMaxFrames);
    EXPECT_EQ(track.activeFrames().size(), expected_active_frames);
    for (const auto &marginalizedKeyframe : track.marginalizedFrames()) {
      EXPECT_TRUE(marginalizedKeyframe->pyramids().empty());
    }
  }

  for (size_t i = 0; i < kFramesToCheck; i++) {
    track.pushFrame(i, time(std::chrono::milliseconds(0)), Sophus::SE3<Precision>());
    expected_active_frames++;
  }
  EXPECT_EQ(track.activeFrames().size(), expected_active_frames);
  marginalizer->marginalize(track);
  EXPECT_EQ(track.activeFrames().size(), kMaxFrames);
}
}  // namespace marginalization
}  // namespace dsopp
