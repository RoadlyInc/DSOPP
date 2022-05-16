#include "feature_based_slam/features/correspondences_finder.hpp"

namespace dsopp {
namespace feature_based_slam {
namespace features {

std::pair<std::unique_ptr<DistinctFeaturesFrame>, std::vector<Correspondence>> findCorrespondences(
    const DistinctFeaturesFrame *features_from, cv::Mat image_from, cv::Mat image_to,
    const sensors::calibration::CameraMask &mask, const features::DistinctFeaturesExtractor &feature_extractor,
    CorrespondencesFinder finder) {
  // reference frame doesn't exist
  if (!features_from) {
    auto reference_features_frame = feature_extractor.extract(image_to, mask, true);
    return {std::move(reference_features_frame), {}};
  }

  return finder(*features_from, image_from, image_to, &feature_extractor, mask);
}

}  // namespace features
}  // namespace feature_based_slam
}  // namespace dsopp
