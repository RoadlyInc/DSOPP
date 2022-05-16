
#ifndef DSOPP_FEATURE_BASED_SLAM_FEATURES_DISTINCT_FEATURES_FRAME_HPP
#define DSOPP_FEATURE_BASED_SLAM_FEATURES_DISTINCT_FEATURES_FRAME_HPP

#include <opencv2/core.hpp>
#include <vector>

namespace dsopp {
namespace feature_based_slam {
namespace features {
class DistinctFeature;
/**
 * \brief DistinctFeaturesFrame is a container for distinct features
 *
 * DistinctFeaturesFrame is a container for distinct features from one frame data.
 */
class DistinctFeaturesFrame {
 public:
  /**
   * creates a features frame from a vector of distinct features.
   * @param distinct_features vector of distinct features
   * @param descriptors descriptors of all features in one matrix
   */
  DistinctFeaturesFrame(std::vector<DistinctFeature> &&distinct_features, cv::Mat descriptors);

  /**
   * @return features of the frame
   */
  const std::vector<DistinctFeature> &features() const;
  ~DistinctFeaturesFrame();

  /**
   * method to get indicator if features descriptors are computed
   * @return indicator
   */
  bool hasDescriptors() const;

  /**
   * method to get all descriptors of all features in one matrix
   * @return descriptors of all features in one matrix
   */
  const cv::Mat descriptors() const;

 private:
  /** vector of distinct features */
  std::vector<DistinctFeature> distinct_features_;
  /** descriptors of all features in one matrix. May be used to match features using opencv matchers */
  cv::Mat descriptors_;
};
}  // namespace features
}  // namespace feature_based_slam
}  // namespace dsopp

#endif  // DSOPP_FEATURE_BASED_SLAM_FEATURES_DISTINCT_FEATURES_FRAME_HPP
