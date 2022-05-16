#ifndef DSOPP_FEATURE_BASED_SLAM_FEATURES_DISTINCT_FEATURE_EXTRACTOR_ORB_HPP_
#define DSOPP_FEATURE_BASED_SLAM_FEATURES_DISTINCT_FEATURE_EXTRACTOR_ORB_HPP_

#include "distinct_features_extractor.hpp"
#include "feature_based_slam/features/distinct_features_extractor.hpp"

#include <opencv2/features2d/features2d.hpp>

namespace dsopp {
namespace feature_based_slam {
namespace features {
/**
 * This extractor extract ORB feature points from frame data and calculate
 * BRISK descriptors for them for future matching of data frames.
 */
class DistinctFeaturesExtractorORB : public DistinctFeaturesExtractor {
 public:
  /**
   * Creating a distinct feature extractor.
   * @param number_of_features number of features to detect
   */
  explicit DistinctFeaturesExtractorORB(size_t number_of_features);
  /**
   * Method to extract distinct features from frame data.
   *
   * @param image image to extract features
   * @param mask input frame mask
   * @param extract_descriptors true if needed to extract descriptors
   * @return distinct features frame which contain distinct features with coordinates and descriptors
   */
  std::unique_ptr<DistinctFeaturesFrame> extract(const cv::Mat &image, const sensors::calibration::CameraMask &mask,
                                                 bool extract_descriptors) const override;

  ~DistinctFeaturesExtractorORB() override;

 private:
  /** feature detector interface */
  cv::Ptr<cv::ORB> orb_detector_;
};
}  // namespace features
}  // namespace feature_based_slam
}  // namespace dsopp

#endif  // DSOPP_FEATURE_BASED_SLAM_FEATURES_DISTINCT_FEATURE_EXTRACTOR_ORB_HPP_
