#include "feature_based_slam/features/distinct_features_extractor_orb.hpp"

#include "common/settings.hpp"
#include "feature_based_slam/features/distinct_feature.hpp"
#include "feature_based_slam/features/distinct_features_frame.hpp"
#include "sensors/camera_calibration/mask/camera_mask.hpp"

namespace dsopp {
namespace feature_based_slam {
namespace features {
DistinctFeaturesExtractorORB::DistinctFeaturesExtractorORB(size_t number_of_features)
    : DistinctFeaturesExtractor(number_of_features) {
  const float kScaleFactor = 2.0;
  const int kPyramidLevels = 5;
  orb_detector_ = cv::ORB::create(static_cast<int>(number_of_features_to_extract_), kScaleFactor, kPyramidLevels);
}

std::unique_ptr<DistinctFeaturesFrame> DistinctFeaturesExtractorORB::extract(
    const cv::Mat &frame_data, const sensors::calibration::CameraMask &mask, bool extract_descriptors) const {
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;

  if (extract_descriptors) {
    orb_detector_->detectAndCompute(frame_data, mask.openCVCompatibleMask(), keypoints, descriptors);
  } else {
    orb_detector_->detect(frame_data, keypoints, mask.openCVCompatibleMask());
  }

  std::vector<DistinctFeature> distinct_features;
  distinct_features.reserve(keypoints.size());

  for (const auto &keypoint : keypoints) {
    Eigen::Vector<Precision, 2> position(keypoint.pt.x, keypoint.pt.y);
    distinct_features.emplace_back(position);
  }

  return std::make_unique<features::DistinctFeaturesFrame>(std::move(distinct_features), descriptors);
}

DistinctFeaturesExtractorORB::~DistinctFeaturesExtractorORB() = default;

}  // namespace features
}  // namespace feature_based_slam
}  // namespace dsopp
