#include "sensors/camera_providers/npy_folder_provider.hpp"

#include <gtest/gtest.h>
#include <string>
#include <vector>

#include "common/file_tools/camera_frame_times.hpp"
#include "common/settings.hpp"

namespace dsopp::sensors::providers {

class testProvider : public ::testing::Test {
 protected:
  void SetUp() override {
    npy_path_ = TEST_DATA_DIR "/track30seconds/CameraSemSeg";
    timestamp_path_ = TEST_DATA_DIR "/track30seconds/times.csv";
    test_frames_count_ = 600;
  }
  void TearDown() override {}

  std::string npy_path_;
  std::string timestamp_path_;
  size_t start_frame_;
  size_t test_frames_count_;
};

TEST_F(testProvider, providerCreatrion) { NpyFolderProvider provider(npy_path_, timestamp_path_, 0); }

TEST_F(testProvider, checkNumberOfFrames) {
  NpyFolderProvider provider(npy_path_, timestamp_path_, 0);
  EXPECT_EQ(test_frames_count_, provider.queueSize());
}

TEST_F(testProvider, shiftStartFrame) {
  for (int start_frame : {0, 5, 10, 15, 20, 25, 30, 35, 36, 38, 100, 200}) {
    NpyFolderProvider provider(npy_path_, timestamp_path_, static_cast<size_t>(start_frame));
    EXPECT_EQ(test_frames_count_ - static_cast<unsigned long>(start_frame), provider.queueSize());
  }
}

TEST_F(testProvider, spendAllFrames) {
  start_frame_ = 0;
  NpyFolderProvider provider(npy_path_, timestamp_path_, start_frame_);

  for (size_t i = 0; i < test_frames_count_; ++i) {
    provider.nextFrame();
    EXPECT_EQ(test_frames_count_ - i - 1, provider.queueSize());
  }
}
}  // namespace dsopp::sensors::providers
