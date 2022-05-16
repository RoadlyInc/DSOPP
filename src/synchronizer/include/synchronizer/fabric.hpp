#ifndef DSOPP_SRC_SYNCHRONIZER_SYNCHRONIZER_FABRIC_HPP_
#define DSOPP_SRC_SYNCHRONIZER_SYNCHRONIZER_FABRIC_HPP_

#include <map>
#include <memory>
#include <string>

namespace dsopp {
namespace sensors {
class Sensors;
}  // namespace sensors
namespace synchronizer {
class Synchronizer;
/**
 * creates Synchronizer with a given configuration
 * @param parameters parameters specific for the synchronizer
 * @param sensors array of system sensors, required to check for the presence of the required sensor
 * @return created synchronizer or nullptr on failure
 */
std::unique_ptr<Synchronizer> create(const std::map<std::string, std::string> &parameters,
                                     const sensors::Sensors &sensors);
}  // namespace synchronizer
}  // namespace dsopp

#endif  // DSOPP_SRC_SYNCHRONIZER_SYNCHRONIZER_FABRIC_HPP_
