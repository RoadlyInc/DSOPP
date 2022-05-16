#ifndef DSOPP_SRC_SENSORS_CAMERA_PROVIDER_FABRIC_HPP_
#define DSOPP_SRC_SENSORS_CAMERA_PROVIDER_FABRIC_HPP_

#include <any>
#include <map>
#include <memory>
#include <string>

namespace dsopp {
namespace sensors {
namespace providers {
class CameraProvider;
/**
 * creates CameraProvider with a given configuration
 * @param parameters parameters specific for the provider
 * @param read_grayscale ``true`` if conversion to grayscale is needed
 * @return created provider or nullptr on failure
 */
std::unique_ptr<CameraProvider> createCameraProvider(const std::map<std::string, std::any> &parameters,
                                                     bool read_grayscale = false);
}  // namespace providers
}  // namespace sensors
}  // namespace dsopp

#endif  // DSOPP_SRC_SENSORS_CAMERA_PROVIDER_FABRIC_HPP_
