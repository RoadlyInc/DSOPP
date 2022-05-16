#include "feature_based_slam/features/orb.hpp"

#include <opencv2/features2d.hpp>

#include "feature_based_slam/features/distinct_feature.hpp"

namespace dsopp::feature_based_slam::features {

std::pair<std::unique_ptr<DistinctFeaturesFrame>, std::vector<Correspondence>> ORBMatch(
    const features::DistinctFeaturesFrame &frame_from, const cv::Mat &, const cv::Mat &image_to,
    const DistinctFeaturesExtractor *extractor, const sensors::calibration::CameraMask &mask) {
  const float kRatioThreshold = 0.7f;

  auto frame_to = extractor->extract(image_to, mask, true);

  const auto descriptors_from = frame_from.descriptors();
  const auto descriptors_to = frame_to->descriptors();

  cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);
  std::vector<std::vector<cv::DMatch>> matches_from_to;
  matcher->knnMatch(descriptors_from, descriptors_to, matches_from_to, 2);

  std::vector<Correspondence> correspondences(matches_from_to.size());

  for (size_t i = 0; i < matches_from_to.size(); i++) {
    correspondences[i].is_inlier = (matches_from_to[i][0].distance < kRatioThreshold * matches_from_to[i][1].distance);
    correspondences[i].idx_from = static_cast<size_t>(matches_from_to[i][0].queryIdx);
    correspondences[i].idx_to = static_cast<size_t>(matches_from_to[i][0].trainIdx);
  }

  return {std::move(frame_to), correspondences};
}

}  // namespace dsopp::feature_based_slam::features
