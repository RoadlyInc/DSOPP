
#include "sensors/camera_providers/image_video_provider.hpp"

#include <gtest/gtest.h>
#include <string>
#include <vector>

#include "common/file_tools/camera_frame_times.hpp"
#include "common/settings.hpp"

namespace dsopp::sensors::providers {

class testProvider : public ::testing::Test {
 protected:
  void SetUp() override {
    video_path_ = TEST_DATA_DIR "/track30seconds/images.mkv";
    timestamp_path_ = TEST_DATA_DIR "/track30seconds/times.csv";
    test_video_frame_count_ = 600;
  }
  void TearDown() override {}

  std::string video_path_;
  std::string timestamp_path_;
  size_t start_frame_;
  size_t test_video_frame_count_;
};

TEST_F(testProvider, providerCreatrion) { ImageVideoProvider provider(video_path_, timestamp_path_, 0); }

TEST_F(testProvider, checkNumberOfFrames) {
  ImageVideoProvider provider(video_path_, timestamp_path_, 0);
  EXPECT_EQ(test_video_frame_count_, provider.queueSize());
}

TEST_F(testProvider, shiftStartFrame) {
  start_frame_ = 100;
  ImageVideoProvider provider(video_path_, timestamp_path_, start_frame_);

  EXPECT_EQ(test_video_frame_count_ - start_frame_, provider.queueSize());
}

TEST_F(testProvider, spendAllFrames) {
  start_frame_ = 0;
  ImageVideoProvider provider(video_path_, timestamp_path_, start_frame_);

  for (size_t i = 0; i < test_video_frame_count_; ++i) provider.nextFrame();

  EXPECT_EQ(0, provider.queueSize());
}

TEST_F(testProvider, losslessReadingTest) {
  std::vector<std::string> image_names = {"00001.png", "00002.png", "00003.png", "00004.png", "00005.png"};
  std::string folder_path = TEST_DATA_DIR "/lossless/";

  ImageVideoProvider provider(folder_path + "lossless.mkv", timestamp_path_, 0);

  for (auto &image_name : image_names) {
    cv::Mat image = cv::imread(folder_path + image_name);
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);

    cv::Mat provider_image = provider.nextFrame()->data();
    cv::cvtColor(provider_image, provider_image, cv::COLOR_BGR2GRAY);

    image.convertTo(image, CV_32FC1);
    provider_image.convertTo(provider_image, CV_32FC1);

    cv::Mat dst = image - provider_image;

    double min, max;
    cv::minMaxIdx(dst, &min, &max);

    double diff = std::max(abs(min), abs(max));

    EXPECT_LE(diff, 1e-8);
  }
}

TEST_F(testProvider, readGrayscale) {
  const bool read_grayscale = true;
  const size_t timestamps_frame_id = 0;
  ImageVideoProvider provider(video_path_, timestamp_path_, 0, test_video_frame_count_, timestamps_frame_id,
                              read_grayscale);
  cv::Mat provider_image = provider.nextFrame()->data();
  EXPECT_EQ(provider_image.type(), CV_8UC1);
}

}  // namespace dsopp::sensors::providers
