#ifndef DSOPP_FEATURE_BASED_SLAM_FEATURES_MATCHER_ORB_MATCHER_HPP
#define DSOPP_FEATURE_BASED_SLAM_FEATURES_MATCHER_ORB_MATCHER_HPP

#include "common/settings.hpp"

#include <Eigen/Dense>
#include <memory>
#include <opencv2/core.hpp>
#include <vector>

#include "feature_based_slam/features/correspondence.hpp"
#include "feature_based_slam/features/distinct_features_extractor.hpp"
#include "feature_based_slam/features/distinct_features_frame.hpp"

namespace dsopp::feature_based_slam::features {

/**
 * ORBMatch finds new positions of the given features on the new frame.
 *
 * @param frame_from starting frame with extracted `DistinctFeatures`
 * @param image_to image from second frame
 * @param extractor will be used to extract new features
 * @param mask mask to avoid new features from the forbidden areas
 * @return extracted features and connections with the frame_from features
 */
std::pair<std::unique_ptr<DistinctFeaturesFrame>, std::vector<Correspondence>> ORBMatch(
    const features::DistinctFeaturesFrame &frame_from, const cv::Mat &, const cv::Mat &image_to,
    const DistinctFeaturesExtractor *extractor, const sensors::calibration::CameraMask &mask);

}  // namespace dsopp::feature_based_slam::features

#endif  // DSOPP_FEATURE_BASED_SLAM_FEATURES_MATCHER_ORB_MATCHER_HPP
