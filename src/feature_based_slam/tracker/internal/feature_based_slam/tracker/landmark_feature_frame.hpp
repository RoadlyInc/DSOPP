#ifndef DSOPP_FEATURE_BASED_SLAM_TRACKER_LANDMARK_FEATURE_FRAME_HPP
#define DSOPP_FEATURE_BASED_SLAM_TRACKER_LANDMARK_FEATURE_FRAME_HPP

#include <memory>

#include "feature_based_slam/features/distinct_feature.hpp"
#include "feature_based_slam/features/distinct_features_frame.hpp"

namespace dsopp {
namespace feature_based_slam {
namespace tracker {
/**
 * Structure for storing conversion of landmarks to feature frame.
 */
struct LandmarkFeatureFrame {
  /** image from which features were taken */
  cv::Mat image;
  /** features obtained from projections of landmarks. Could be nullptr, if there are no landmarks */
  std::unique_ptr<features::DistinctFeaturesFrame> features = nullptr;
  /** correspondences between feature and landmark */
  std::vector<size_t> landmark_idx;
};

}  // namespace tracker
}  // namespace feature_based_slam
}  // namespace dsopp

#endif  // DSOPP_FEATURE_BASED_SLAM_TRACKER_LANDMARK_FEATURE_FRAME_HPP
