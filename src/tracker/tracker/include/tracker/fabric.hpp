
#ifndef DSOPP_TRACKER_FABRIC_HPP
#define DSOPP_TRACKER_FABRIC_HPP

#include <any>
#include <map>
#include <memory>

#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"
#include "energy/problems/photometric_bundle_adjustment/photometric_bundle_adjustment.hpp"
#include "tracker/depth_estimators/depth_estimator.hpp"

namespace dsopp {
namespace sensors {
class Sensors;
}  // namespace sensors
namespace tracker {
template <energy::motion::Motion Motion>
class Tracker;
/**
 * creates Tracker with a given configuration
 * @param parameters parameters specific for the Tracker
 * @param sensors array of system sensors, required to check for the presence of the required sensor
 * @return created tracker or nullptr on failure
 */
template <energy::motion::Motion Motion, typename DepthEstimator, template <int> typename Grid2D>
requires tracker::DepthEstimator<DepthEstimator, typename Motion::Product> std::unique_ptr<Tracker<Motion>> create(
    const std::map<std::string, std::any> &parameters, const sensors::Sensors &sensors);
}  // namespace tracker
}  // namespace dsopp

#endif  // DSOPP_TRACKER_FABRIC_HPP
