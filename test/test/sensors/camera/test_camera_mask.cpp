
#include "sensors/camera_calibration/mask/camera_mask.hpp"

#include "common/file_tools/camera_frame_times.hpp"
#include "common/settings.hpp"
#include "common/time/time.hpp"
#include "feature_based_slam/features/distinct_feature.hpp"
#include "feature_based_slam/features/distinct_features_extractor_orb.hpp"
#include "feature_based_slam/features/distinct_features_frame.hpp"
#include "features/camera/pixel_data_frame.hpp"
#include "features/camera/pixel_map.hpp"
#include "features/camera/sobel_tracking_features_extractor.hpp"
#include "features/camera/tracking_feature.hpp"
#include "features/camera/tracking_features_frame.hpp"
#include "sensors/camera_calibration/fabric.hpp"
#include "sensors/camera_providers/camera_data_frame.hpp"
#include "sensors/camera_providers/image_video_provider.hpp"

#include <gtest/gtest.h>
#include <Eigen/Dense>

namespace dsopp {
namespace features {
TEST(trackingFeaturesExtractor, trackingFeaturesExtractor) {
  SobelTrackingFeaturesExtractor extractor;

  auto provider = std::make_unique<sensors::providers::ImageVideoProvider>(TEST_DATA_DIR "track30seconds/images.mkv",
                                                                           TEST_DATA_DIR "track30seconds/times.csv");
  cv::Mat pixel_image = provider->nextFrame()->data();

  cv::Mat mask = cv::imread(TEST_DATA_DIR "track30seconds/test_mask.png", 0);
  auto camera_mask = sensors::calibration::CameraMask(mask);

  auto tracking_frame = extractor.extract(pixel_image, camera_mask);
  auto &features = tracking_frame->features();

  const size_t kBorderSize = 4;
  camera_mask = camera_mask.getEroded(kBorderSize);

  for (auto &pt : features) {
    auto coord = pt.coordinates();
    EXPECT_TRUE(camera_mask.valid(coord.x(), coord.y()));
  }
}

TEST(DistinctFeaturesExtractor, DistinctFeaturesExtractor) {
  feature_based_slam::features::DistinctFeaturesExtractorORB extractor(1000);

  auto provider = std::make_unique<sensors::providers::ImageVideoProvider>(TEST_DATA_DIR "track30seconds/images.mkv",
                                                                           TEST_DATA_DIR "track30seconds/times.csv");
  cv::Mat image = provider->nextFrame()->data();

  cv::Mat mask = cv::imread(TEST_DATA_DIR "track30seconds/test_mask.png", 0);
  auto camera_mask = sensors::calibration::CameraMask(mask);

  auto distinct_frame = extractor.extract(image, camera_mask, false);
  auto &features = distinct_frame->features();

  for (auto &pt : features) {
    auto coord = pt.coordinates();
    EXPECT_TRUE(camera_mask.valid(coord.x(), coord.y()));
  }
}

}  // namespace features
}  // namespace dsopp
