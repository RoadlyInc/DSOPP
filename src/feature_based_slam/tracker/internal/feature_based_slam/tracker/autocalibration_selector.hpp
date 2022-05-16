#ifndef DSOPP_FEATURE_BASED_SLAM_TRACKER_AUTOCALIBRATION_SELECTOR_HPP
#define DSOPP_FEATURE_BASED_SLAM_TRACKER_AUTOCALIBRATION_SELECTOR_HPP

#include "common/settings.hpp"

#include <Eigen/Dense>

namespace dsopp {
namespace feature_based_slam {
namespace tracker {
/**
 * Stores result of autocalibration process and decides which parameters choose
 */
class AutocalibrationSelector {
 public:
  /**
   * Add new parameters after calibration
   * @param focal_length, k estimated camera parameters
   */
  void addResult(Precision focal_length, const Eigen::Vector2<Precision> &k = Eigen::Vector2<Precision>::Zero());
  /**
   * Clear all stored parameters
   */
  void reset();
  /**
   * @return currently optimal focal length
   */
  Precision getFocalLength();

  /**
   * @return currently optimal distortion coefficients
   */
  Eigen::Vector2<Precision> getDistortionCoeffs();

  /**
   * @return number of results stored in the selector
   */
  size_t size() const;

 private:
  /** stored focal lengths */
  std::vector<Precision> focal_length_variants_;
  /** stored first distortion coefficient */
  std::vector<Precision> k1_variants_;
  /** stored second distortion coefficient */
  std::vector<Precision> k2_variants_;
};
}  // namespace tracker
}  // namespace feature_based_slam
}  // namespace dsopp

#endif  // DSOPP_FEATURE_BASED_SLAM_TRACKER_AUTOCALIBRATION_SELECTOR_HPP
