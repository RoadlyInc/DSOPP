#include "features/camera/sobel_tracking_features_extractor.hpp"

#include "common/file_tools/camera_frame_times.hpp"
#include "common/time/time.hpp"
#include "features/camera/pixel_data_frame.hpp"
#include "features/camera/pixel_map.hpp"
#include "features/camera/tracking_feature.hpp"
#include "features/camera/tracking_features_frame.hpp"
#include "sensors/camera_calibration/fabric.hpp"
#include "sensors/camera_calibration/mask/camera_mask.hpp"
#include "sensors/camera_providers/camera_data_frame.hpp"
#include "sensors/camera_providers/camera_provider.hpp"
#include "sensors/camera_providers/image_video_provider.hpp"

#include <gtest/gtest.h>
#include <Eigen/Dense>

namespace dsopp {
namespace features {
TEST(tracking_features_extractor, tracking_features_extractor_determinacy) {
  auto provider = std::make_unique<sensors::providers::ImageVideoProvider>(
      TEST_DATA_DIR "track30seconds/images.mkv", TEST_DATA_DIR "track30seconds/times.csv", 20, 30, 0, true);

  cv::Mat image = provider->nextFrame()->data();

  sensors::calibration::CameraMask mask(image.rows, image.cols);

  const size_t kTestN = 10;

  std::vector<std::unique_ptr<TrackingFeaturesFrame>> features_frames;

  for (size_t iteration = 0; iteration < kTestN; ++iteration) {
    SobelTrackingFeaturesExtractor extractor;
    features_frames.push_back(extractor.extract(image, mask));
  }

  for (const auto &features_frame : features_frames) {
    auto &first_frame = features_frames[0]->features();
    auto &frame = features_frame->features();
    EXPECT_EQ(first_frame.size(), frame.size());
    for (size_t i = 0; i < first_frame.size(); i++) {
      EXPECT_TRUE(first_frame[i].coordinates() == frame[i].coordinates());
    }
  }
}
}  // namespace features
}  // namespace dsopp
