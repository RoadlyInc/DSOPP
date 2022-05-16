#include "agent/agent_settings.hpp"

#include <glog/logging.h>

#include "camera_settings.pb.h"
#include "sensors/camera/camera.hpp"

#include "sensors/sensors.hpp"

namespace dsopp::sensors {
namespace {

std::optional<calibration::proto::CameraSettings::ModelType> modelTypeToProtoType(energy::model::ModelType type) {
  if (type == energy::model::ModelType::kPinholeCamera) return calibration::proto::CameraSettings::PINHOLE;
  if (type == energy::model::ModelType::kSimpleRadialCamera) return calibration::proto::CameraSettings::SIMPLE_RADIAL;
  LOG(WARNING) << "Type could not be processed (" << static_cast<int>(type) << ")";

  return {};
}

bool isCameraType(calibration::proto::CameraSettings::ModelType proto_type) {
  if (proto_type == calibration::proto::CameraSettings::PINHOLE ||
      proto_type == calibration::proto::CameraSettings::SIMPLE_RADIAL) {
    return true;
  }
  LOG(WARNING) << "Type could not be processed (" << static_cast<int>(proto_type) << ")";
  return false;
}

}  // namespace
AgentSettings::AgentSettings(AgentSettings &&) = default;
AgentSettings::AgentSettings() = default;

calibration::CameraSettings &AgentSettings::addCameraSettings(calibration::CameraSettings &&camera_settings) {
  return camera_settings_.emplace(camera_settings_.size(), std::move(camera_settings)).first->second;
}

AgentSettings::AgentSettings(const proto::AgentSettings &proto) {
  for (auto &[sensor_id, camera_settings_proto] : proto.camera_settings()) {
    if (isCameraType(camera_settings_proto.model_type())) {
      camera_settings_.emplace(sensor_id, calibration::CameraSettings(camera_settings_proto));
    }
  }
}

proto::AgentSettings AgentSettings::proto() const {
  proto::AgentSettings agent_settings;

  for (auto &[sensor_id, camera] : camera_settings_) {
    auto proto_type = modelTypeToProtoType(camera.calibration().type());
    if (!proto_type) continue;

    auto camera_proto = camera.proto();
    camera_proto.set_model_type(*proto_type);
    (*agent_settings.mutable_camera_settings())[sensor_id] = camera_proto;
  }
  return agent_settings;
}
AgentSettings::AgentSettings(const AgentSettings &other) {
  for (const auto &pair : other.camera_settings_) {
    camera_settings_.emplace(pair.first, pair.second.clone());
  }
}
AgentSettings AgentSettings::clone() const { return *this; }

const std::map<size_t, calibration::CameraSettings> &AgentSettings::cameraSettings() const { return camera_settings_; }
}  // namespace dsopp::sensors
