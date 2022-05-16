#ifndef DSOPP_FEATURE_BASED_SLAM_FEATURES_CORRESPONDENCE_HPP
#define DSOPP_FEATURE_BASED_SLAM_FEATURES_CORRESPONDENCE_HPP

#include "common/settings.hpp"

namespace dsopp {
namespace feature_based_slam {
namespace features {
/**
 * \brief Correspondence between two features from the two frames
 */
struct Correspondence {
  /** true if the correspondence is inlier and false otherwise */
  bool is_inlier;
  /** index of the feature from the `from` frame */
  size_t idx_from;
  /** index of the feature from the `to` frame */
  size_t idx_to;
};
}  // namespace features
}  // namespace feature_based_slam
}  // namespace dsopp

#endif  // DSOPP_FEATURE_BASED_SLAM_FEATURES_CORRESPONDENCE_HPP
