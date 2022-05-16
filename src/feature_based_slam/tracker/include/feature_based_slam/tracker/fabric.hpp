#ifndef DSOPP_FEATURE_BASED_SLAM_TRACKER_FABRIC_HPP_
#define DSOPP_FEATURE_BASED_SLAM_TRACKER_FABRIC_HPP_

#include <any>
#include <map>
#include <memory>
#include <string>

#include "feature_based_slam/tracker/tracker.hpp"
#include "sensors/sensors.hpp"

namespace dsopp::feature_based_slam::tracker {

/**
 * cretes feature based slam tracker from the given parameters
 *
 * @tparam Calibration camera calibration type
 *
 * @param parameters parameters
 * @param sensors array of system sensors, required to check for the presence of the required sensor
 * @return unique pointer to feature based slam tracker object
 */
std::unique_ptr<Tracker> create(const std::map<std::string, std::any> &parameters, const sensors::Sensors &sensors);

}  // namespace dsopp::feature_based_slam::tracker

#endif  // DSOPP_FEATURE_BASED_SLAM_TRACKER_FABRIC_HPP_
