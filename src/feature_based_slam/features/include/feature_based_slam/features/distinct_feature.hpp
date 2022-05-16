
#ifndef DSOPP_FEATURE_BASED_SLAM_FEATURES_DISTINCT_FEATURE_HPP
#define DSOPP_FEATURE_BASED_SLAM_FEATURES_DISTINCT_FEATURE_HPP

#include <bitset>

#include <Eigen/Dense>

#include "common/settings.hpp"

namespace dsopp {
namespace feature_based_slam {
namespace features {
/**
 * \brief DistinctFeature represent an identifiable feature.
 *
 * Object of this class stores coordinates and descriptor of the distinct feature.
 */
class DistinctFeature {
 public:
  /**
   * creates a distinct feature from coordinates.
   * @param coordinates coordinates of the distinct feature
   */
  DistinctFeature(const Eigen::Vector<Precision, 2> &coordinates);
  /**
   * @return coordinates of the point
   */
  const Eigen::Vector<Precision, 2> &coordinates() const;

  ~DistinctFeature();

 private:
  /** feature coordinates*/
  Eigen::Vector<Precision, 2> coordinates_;
};
}  // namespace features
}  // namespace feature_based_slam
}  // namespace dsopp

#endif  // DSOPP_FEATURE_BASED_SLAM_FEATURES_DISTINCT_FEATURE_HPP
