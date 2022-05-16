
#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include "common/settings.hpp"
#include "sensors/camera_providers/image_folder_provider.hpp"
namespace dsopp {
namespace sensors {
namespace providers {

class testProvider : public ::testing::Test {
 protected:
  void SetUp() override {
    clearData();
    test_path_ = std::string(GENERATED_TEST_DATA_DIR) + "/test_image_folder";
    test_timestamps_file_ = std::string(GENERATED_TEST_DATA_DIR) + "/test_image_folder/timestamps.csv";
    batch_size_ = 2;
    // create temp directory
    std::filesystem::create_directories(test_path_);
  }
  void clearData() {
    // remove temp directory
    std::filesystem::remove_all(test_path_);
  }
  void TearDown() override { clearData(); }
  void createImage(const std::string &file_name) {
    cv::Mat image(50, 100, CV_8UC3, cv::Scalar(255, 0, 0));
    cv::imwrite(test_path_ + "/" + file_name, image);
  }
  void createImage(int frame_id, const std::string &extension) {
    cv::Mat image(50, 100, CV_8UC3, cv::Scalar(255, 0, 0));
    cv::imwrite(test_path_ + "/" + std::to_string(frame_id) + extension, image);
    std::ofstream file;
    file.open(test_timestamps_file_, std::ios_base::app);
    file << frame_id << " 0" << std::endl;
    file.close();
  }
  std::string test_path_;
  std::string test_timestamps_file_;
  size_t batch_size_;
};

TEST_F(testProvider, emptyFolder) {
  ImageFolderProvider provider(test_path_, test_timestamps_file_, batch_size_);
  ASSERT_EQ(provider.queueSize(), 0);
  ASSERT_EQ(provider.nextFrame(), nullptr);
}

TEST_F(testProvider, folderWithOneImage) {
  int frame_id = 1;
  createImage(frame_id, ".jpg");
  ImageFolderProvider provider(test_path_, test_timestamps_file_, batch_size_);
  ASSERT_EQ(provider.queueSize(), 1);
  ASSERT_EQ(provider.nextFrame()->id(), frame_id);
  ASSERT_EQ(provider.queueSize(), 0);
  ASSERT_EQ(provider.nextFrame(), nullptr);
}

TEST_F(testProvider, folderWithThreeImages) {
  int frame_id1 = 1;
  int frame_id2 = 4;
  int frame_id3 = 7;
  createImage(frame_id1, ".jpg");
  createImage(frame_id2, ".bmp");
  createImage(frame_id3, ".jpg");
  ImageFolderProvider provider(test_path_, test_timestamps_file_, batch_size_);
  ASSERT_EQ(provider.queueSize(), 3);
  ASSERT_EQ(provider.nextFrame()->id(), frame_id1);
  ASSERT_EQ(provider.queueSize(), 2);
  ASSERT_EQ(provider.nextFrame()->id(), frame_id2);
  ASSERT_EQ(provider.queueSize(), 1);
  ASSERT_EQ(provider.nextFrame()->id(), frame_id3);
  ASSERT_EQ(provider.queueSize(), 0);
  ASSERT_EQ(provider.nextFrame(), nullptr);
}

TEST_F(testProvider, folderWithImageAndTrash) {
  int frame_id = 1;
  createImage(frame_id, ".jpg");
  createImage(frame_id, ".png");
  ImageFolderProvider provider(test_path_, test_timestamps_file_, batch_size_);
  ASSERT_EQ(provider.queueSize(), 1);
  ASSERT_EQ(provider.nextFrame()->id(), frame_id);
  ASSERT_EQ(provider.queueSize(), 0);
  ASSERT_EQ(provider.nextFrame(), nullptr);
}

TEST_F(testProvider, nonexistentFolder) {
  EXPECT_THROW(ImageFolderProvider(test_path_ + "abc", test_timestamps_file_, batch_size_),
               std::filesystem::filesystem_error);
}

TEST_F(testProvider, folderWithImageWithTheWrongName) {
  createImage("abc.jpg");
  EXPECT_THROW(ImageFolderProvider(test_path_, test_timestamps_file_, batch_size_), std::invalid_argument);
}

TEST_F(testProvider, readGrayscale) {
  int frame_id = 1;
  createImage(frame_id, ".jpg");
  createImage(frame_id, ".png");
  const bool read_grayscale = true;
  ImageFolderProvider provider(test_path_, test_timestamps_file_, batch_size_, 0, 10, read_grayscale);
  cv::Mat provider_image = provider.nextFrame()->data();
  EXPECT_EQ(provider_image.type(), CV_8UC1);
}

}  // namespace providers
}  // namespace sensors
}  // namespace dsopp
