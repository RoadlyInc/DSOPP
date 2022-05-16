#ifndef DSOPP_DSOPP_DRIVE_AGENT_SETTINGS_HPP_
#define DSOPP_DSOPP_DRIVE_AGENT_SETTINGS_HPP_

#include <memory>

#include "agent/agent.hpp"
#include "agent_settings.pb.h"
#include "sensors/camera_calibration/camera_calibration.hpp"
#include "sensors/camera_calibration/camera_settings.hpp"

namespace dsopp::sensors {

/**
 * \brief Agent settings, stores all information to construct agent
 */
class AgentSettings {
 public:
  /**
   * create empty AgentSettings to add sensor later
   */
  AgentSettings();
  /**
   * add new camera settings
   * @param camera_settings camera settings to store
   * @return reference to the new locatiion of camera settings
   */
  calibration::CameraSettings &addCameraSettings(calibration::CameraSettings &&camera_settings);
  /**
   * @param proto protobuf message
   */
  AgentSettings(const proto::AgentSettings &proto);
  /**
   * move constructor
   */
  AgentSettings(AgentSettings &&);
  /**
   * @return protobuf message
   */
  proto::AgentSettings proto() const;
  /**
   * @return return camera settings correspond to the id
   */
  const std::map<size_t, calibration::CameraSettings> &cameraSettings() const;
  /**
   * @return deep copy of the object
   */
  AgentSettings clone() const;

 protected:
  /**
   * @return deep copy of the object
   */
  AgentSettings(const AgentSettings &);

 private:
  /** sensor_id -> Camera map */
  std::map<size_t, calibration::CameraSettings> camera_settings_;
};

}  // namespace dsopp::sensors

#endif  // DSOPP_DSOPP_DRIVE_AGENT_SETTINGS_HPP_
