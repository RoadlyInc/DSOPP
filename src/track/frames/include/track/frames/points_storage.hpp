#ifndef DSOPP_TRACK_FRAMES_POINTS_STORAGE_HPP_
#define DSOPP_TRACK_FRAMES_POINTS_STORAGE_HPP_

#include <Eigen/Dense>
#include <map>
#include <vector>

#include "common/settings.hpp"

namespace dsopp::track::storage {

/** point type */
enum struct PointStorageType : uint8_t {
  kActive = 0,       /**< active type. */
  kMarginalized = 1, /**< marginalized type. */
  kOutlier = 2       /**< outlier type. */
};

/** points with colors from the frame*/
struct PointsStorage {
  /** points of the keyframe */
  std::map<size_t, std::vector<Eigen::Vector<Precision, 3>>> points;
  /** colors of points */
  std::map<size_t, std::vector<Eigen::Vector3<uint8_t>>> colors;
  /** default colors of points */
  std::map<size_t, std::vector<Eigen::Vector3<uint8_t>>> default_colors;
  /** semantic colors of points */
  std::map<size_t, std::vector<Eigen::Vector3<uint8_t>>> semantics_colors;
  /** variance of inverse depth for all points */
  std::map<size_t, std::vector<Precision>> idepth_variances;
  /** relative baseline of points */
  std::map<size_t, std::vector<Precision>> relative_baselines;
};

}  // namespace dsopp::track::storage

#endif  // DSOPP_TRACK_FRAMES_POINTS_STORAGE_HPP_
