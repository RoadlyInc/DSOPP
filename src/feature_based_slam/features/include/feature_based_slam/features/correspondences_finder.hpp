#ifndef DSOPP_FEATURE_BASED_SLAM_FEATURES_INITIALIZATION_MATCHER_HPP
#define DSOPP_FEATURE_BASED_SLAM_FEATURES_INITIALIZATION_MATCHER_HPP

#include <vector>

#include "feature_based_slam/features/correspondence.hpp"
#include "feature_based_slam/features/distinct_features_extractor.hpp"
#include "feature_based_slam/features/distinct_features_frame.hpp"

namespace dsopp {
namespace feature_based_slam {
namespace features {

/** Extract feature and connect its with features from the reference
 * frame. Return unique_ptr with new features and vector of connections with the given frame. */
using CorrespondencesFinder =
    std::function<std::pair<std::unique_ptr<DistinctFeaturesFrame>, std::vector<Correspondence>>(
        const features::DistinctFeaturesFrame &, const cv::Mat &, const cv::Mat &, const DistinctFeaturesExtractor *,
        const sensors::calibration::CameraMask &)>;

/**
 * Function to extract features and connect its with the given features. If features are not given, only
 * extract features
 * @param features_from features to find correspondences. Can be nullptr.
 * @param image_from reference image.
 * @param image_to image on which new features will be founded
 * @param mask mask to avoid features from the forbidden areas
 * @param finder create correspondences from the reference image to the given
 * @param feature_extractor defines the type of the features that would be extracted (ORB, BRISK, etc.)
 * @return features and connections with the first frame
 */

std::pair<std::unique_ptr<DistinctFeaturesFrame>, std::vector<Correspondence>> findCorrespondences(
    const DistinctFeaturesFrame *features_from, cv::Mat image_from, cv::Mat image_to,
    const sensors::calibration::CameraMask &mask, const features::DistinctFeaturesExtractor &feature_extractor,
    CorrespondencesFinder finder);
}  // namespace features
}  // namespace feature_based_slam
}  // namespace dsopp

#endif  // DSOPP_FEATURE_BASED_SLAM_FEATURES_INITIALIZATION_MATCHER_HPP
