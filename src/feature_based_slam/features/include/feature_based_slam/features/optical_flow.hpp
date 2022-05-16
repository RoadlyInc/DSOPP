#ifndef DSOPP_FEATURE_BASED_SLAM_FEATURES_MATCHER_OPTICAL_FLOW_HPP_
#define DSOPP_FEATURE_BASED_SLAM_FEATURES_MATCHER_OPTICAL_FLOW_HPP_

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
 * OpticalFlowMatch finds new positions of the given features on the new frame.
 *
 * @param frame_from starting optical flow frame with extracted `DistinctFeatures`
 * @param image_from image from first frame
 * @param image_to image from second frame
 * @param features_extractor needs to add new landmarks, if founded points number less than the reference
 * @param mask mask to find new points
 * @return extracted features and connections with the frame_from features
 */
std::pair<std::unique_ptr<DistinctFeaturesFrame>, std::vector<Correspondence>> OpticalFlowMatch(
    const features::DistinctFeaturesFrame &frame_from, const cv::Mat &image_from, const cv::Mat &image_to,
    const DistinctFeaturesExtractor *features_extractor, const sensors::calibration::CameraMask &mask);

}  // namespace dsopp::feature_based_slam::features

#endif  // DSOPP_FEATURE_BASED_SLAM_FEATURES_MATCHER_OPTICAL_FLOW_HPP_
