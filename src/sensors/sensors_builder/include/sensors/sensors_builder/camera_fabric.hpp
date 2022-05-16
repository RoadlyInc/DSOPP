
#ifndef DSOPP_SENSOR_FABRIC_HPP
#define DSOPP_SENSOR_FABRIC_HPP

#include <any>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace dsopp {
namespace sensors {
namespace camera_transformers {
class CameraTransformer;
}
namespace calibration {
class CameraSettings;
}
class Camera;
/**
 * creates Camera with a given configuration
 * @param parameters parameters specific for the sensor and sensor provider
 * @param sensor_name unique name for sensor
 * @param sensor_id unique identifier for sensor
 * @return created camera or nullptr on failure
 */
std::unique_ptr<Camera> createCamera(
    const std::map<std::string, std::any> &parameters, const std::string &sensor_name, size_t sensor_id,
    std::vector<std::unique_ptr<camera_transformers::CameraTransformer>> &&transformers,
    const calibration::CameraSettings &camera_settings);
/**
 * creates CameraSettings with a given configuration
 * @param parameters parameters specific for the sensor and sensor provider
 * @return create camera settings or nullptr on failure
 */
template <bool TRANSFORM_TO_PINHOLE>
std::optional<calibration::CameraSettings> createCameraSettings(
    const std::map<std::string, std::any> &parameters,
    const std::vector<std::unique_ptr<camera_transformers::CameraTransformer>> &transformers);
/**
 * creates vector of CameraTransformers with a given configuration
 * @return vector of CameraTransformers
 */
std::vector<std::unique_ptr<camera_transformers::CameraTransformer>> createCameraTransformer(
    const std::map<std::string, std::any> &parameters);
}  // namespace sensors
}  // namespace dsopp
#endif  // DSOPP_SENSOR_FABRIC_HPP
