
#ifndef DSOPP_SRC_SENSORS_CAMERA_CALIBRATION_FABRIC_HPP_
#define DSOPP_SRC_SENSORS_CAMERA_CALIBRATION_FABRIC_HPP_

#include <any>
#include <map>
#include <memory>
#include <string>

#include <opencv2/core.hpp>
#include "common/settings.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"
#include "sensors/camera_calibration/mask/camera_mask.hpp"

namespace dsopp {
namespace sensors {
namespace calibration {
class PinholeCameraCalibration;
class SimpleRadialCalibration;

namespace camera_calibration {
/**
 * creates CameraCalibration with a given configuration
 * @param parameters parameters specific for the calibration
 * @return created calibration or nullptr on failure
 */
std::optional<CameraCalibration> create(const std::map<std::string, std::any> &parameters,
                                        bool transform_to_pinhole = true);
}  // namespace camera_calibration

namespace photometric_calibration {
/** Photometric calibration size */
constexpr size_t kPhotometricCalibrationPreimageSize = 256;
/** Alias for photometric calibration */
using PhotometricCalibration = std::array<Precision, kPhotometricCalibrationPreimageSize>;

/**
 * creates photometric calibration from file
 * @param photometric_calibration_file path to the file with photometric calibration
 * @return created photometric calibration
 */
PhotometricCalibration create(const std::string &photometric_calibration_file);

/**
 * creates photometric calibration with a given configuration
 * @param parameters parameters specific for the photometric calibration
 * @return created photometric calibration
 */
PhotometricCalibration create(const std::map<std::string, std::any> &parameters);
}  // namespace photometric_calibration

namespace vignetting {
/**
 * creates vignetting from image
 * @param vignetting_path path to the image with vignetting
 * @return created vignetting
 */
cv::Mat create(const std::string &vignetting_path);

/**
 * creates vignetting with a given configuration
 * @param parameters parameters specific for the vignetting
 * @return created vignetting
 */
cv::Mat create(const std::map<std::string, std::any> &parameters);
}  // namespace vignetting
namespace mask {
std::optional<sensors::calibration::CameraMask> create(const std::map<std::string, std::any> &parameters, int width,
                                                       int height);
}
}  // namespace calibration
}  // namespace sensors
}  // namespace dsopp
#endif  // DSOPP_SRC_SENSORS_CAMERA_CALIBRATION_FABRIC_HPP_
