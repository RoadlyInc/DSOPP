#include "feature_based_slam/features/optical_flow.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>
#include <random>

#include "feature_based_slam/features/distinct_feature.hpp"

namespace dsopp::feature_based_slam::features {

std::pair<std::unique_ptr<DistinctFeaturesFrame>, std::vector<Correspondence>> OpticalFlowMatch(
    const features::DistinctFeaturesFrame &frame_from, const cv::Mat &image_from, const cv::Mat &image_to,
    const DistinctFeaturesExtractor *features_extractor, const sensors::calibration::CameraMask &mask) {
  std::vector<cv::Point2f> frame_from_features, frame_to_features;

  frame_from_features.reserve(frame_from.features().size());
  frame_to_features.reserve(frame_from.features().size());

  const int kWindowSize = 15;
  const cv::Size neighbour_window_(kWindowSize, kWindowSize);
  const int max_pyramid_level_ = 3;
  const cv::TermCriteria termination_criteria_((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 10, 0.01);

  for (const auto &pt : frame_from.features()) {
    frame_from_features.emplace_back(pt.coordinates().x(), pt.coordinates().y());
  }

  std::vector<uchar> status;
  std::vector<float> err;
  cv::calcOpticalFlowPyrLK(image_from, image_to, frame_from_features, frame_to_features, status, err, neighbour_window_,
                           max_pyramid_level_, termination_criteria_);

  std::vector<Correspondence> correspondences;
  std::vector<DistinctFeature> features;

  for (size_t i = 0; i < frame_from.features().size(); ++i) {
    if (status[i] != 0) {
      // create feature
      features.emplace_back(Eigen::Vector2<Precision>(frame_to_features[i].x, frame_to_features[i].y));
      correspondences.push_back({true, i, features.size() - 1});
    }
  }

  // generate new features and get missing quantity from random place
  auto new_features = features_extractor->extract(image_to, mask, false);
  auto rng = std::default_random_engine{};
  std::vector<size_t> indices(new_features->features().size());
  std::iota(indices.begin(), indices.end(), 0);
  std::shuffle(indices.begin(), indices.end(), rng);

  for (size_t i = 0; features.size() < features_extractor->featureCount() && i < indices.size(); i++) {
    features.push_back(new_features->features()[indices[i]]);
  }

  return {std::make_unique<DistinctFeaturesFrame>(std::move(features), cv::Mat()), correspondences};
}

}  // namespace dsopp::feature_based_slam::features
