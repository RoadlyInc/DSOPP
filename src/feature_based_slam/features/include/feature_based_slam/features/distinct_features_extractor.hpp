#ifndef DSOPP_FEATURE_BASED_SLAM_FEATURES_DISTINCT_FEATURE_EXTRACTOR_HPP_
#define DSOPP_FEATURE_BASED_SLAM_FEATURES_DISTINCT_FEATURE_EXTRACTOR_HPP_

#include <memory>
#include <opencv2/opencv.hpp>

namespace dsopp {
namespace sensors {
namespace calibration {
class CameraMask;
}
}  // namespace sensors
namespace feature_based_slam {
namespace features {
class DistinctFeaturesFrame;
/**
 * \brief DistinctFeatureExtractor extract distinct feature points from frame data.
 *
 */
class DistinctFeaturesExtractor {
 public:
  /**
   * Create DistinctFeatureExtractor with the given number of extracted features
   * @param number_of_features_to_extract
   */
  DistinctFeaturesExtractor(size_t number_of_features_to_extract)
      : number_of_features_to_extract_(number_of_features_to_extract) {}
  /**
   * Method to extract distinct features from frame data.
   *
   * @param image image to extract features
   * @param mask input frame mask
   * @param extract_descriptors true if needed to extract descriptors
   * @return distinct features frame which contain distinct features with coordinates and descriptors
   */
  virtual std::unique_ptr<DistinctFeaturesFrame> extract(const cv::Mat &image,
                                                         const sensors::calibration::CameraMask &mask,
                                                         bool extract_descriptors) const = 0;

  /**
   * @return number of features that will be extracted
   */
  virtual size_t featureCount() const { return number_of_features_to_extract_; }

  virtual ~DistinctFeaturesExtractor() = default;

 protected:
  /** number of features that will be extracted */
  const size_t number_of_features_to_extract_;
};
}  // namespace features
}  // namespace feature_based_slam
}  // namespace dsopp

#endif  // DSOPP_FEATURE_BASED_SLAM_FEATURES_DISTINCT_FEATURE_EXTRACTOR_HPP_
