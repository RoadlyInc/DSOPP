#ifndef SENSORS_CAMERA_TRANSFORMERS_FABRIC_HPP_
#define SENSORS_CAMERA_TRANSFORMERS_FABRIC_HPP_

#include <any>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "sensors/camera_transformers/camera_transformer.hpp"

namespace dsopp::sensors::camera_transformers {

/**
 *
 * @tparam Calibration camera calibration type
 * @param parameters map of parameters
 * @return vector of image transformers
 */
std::vector<std::unique_ptr<CameraTransformer>> create(const std::map<std::string, std::any> &parameters);
}  // namespace dsopp::sensors::camera_transformers

#endif  // SENSORS_CAMERA_TRANSFORMERS_FABRIC_HPP_
