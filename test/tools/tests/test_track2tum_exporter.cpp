#include "track/export/track2tum_exporter.hpp"

#include "common/settings.hpp"
#include "test/tools/random_track.hpp"
#include "test/tools/tum_gt.hpp"
#include "track/frames/tracking_frame.hpp"
#include "track/odometry_track.hpp"

#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>

namespace dsopp {
namespace {
void assertEqual(const Sophus::SE3<Precision> &t_1, const Sophus::SE3<Precision> &t_2) {
  ASSERT_LE((t_1 * t_2.inverse()).log().norm(), 1e-10);
}
}  // namespace

namespace test_tools {
class testTrack2TumExporter : public ::testing::Test {
 protected:
  void SetUp() override { test_path_ = std::string(GENERATED_TEST_DATA_DIR) + "/random_track.tum"; }

  void TearDown() override {
    // remove temporary file
    std::filesystem::remove(test_path_);
  }

  std::string test_path_;
};

TEST_F(testTrack2TumExporter, randomTrack) {
  const auto &track = randomTrack();
  track2TumExporter(*track, test_path_);
  auto tum_gt = TumGt(test_path_);

  for (const auto &frame : track->keyframes()) {
    assertEqual(frame->tWorldAgent(), tum_gt.getPose(frame->timestamp()));
    for (const auto &attached_frame : frame->attachedFrames()) {
      assertEqual(attached_frame->tWorldAgent(), tum_gt.getPose(attached_frame->timestamp()));
    }
  }
}
}  // namespace test_tools
}  // namespace dsopp
