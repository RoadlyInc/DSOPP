#include "synchronizer/fabric.hpp"

#include <glog/logging.h>

#include "sensor/sensor.hpp"
#include "sensors/sensors.hpp"
#include "synchronizer/master_sensor_synchronizer.hpp"
#include "synchronizer/no_synchronization.hpp"

namespace dsopp {
namespace synchronizer {
std::unique_ptr<Synchronizer> create(const std::map<std::string, std::string> &parameters,
                                     const sensors::Sensors &sensors) {
  if (parameters.count("type") == 0) {
    LOG(WARNING) << "Missing field \"type\" in the time parameters. Synchronizer was not created.";
    return nullptr;
  }

  auto &time_type = parameters.at("type");

  if (time_type == "master") {
    if (parameters.count("sensor_id") == 0) {
      LOG(WARNING) << "Missing field \"sensor_id\" in the time parameters. Synchronizer was not created.";
      return nullptr;
    }
    auto &time_sensor_name = parameters.at("sensor_id");

    auto *sensor = sensors.get(time_sensor_name);

    if (!sensor) {
      LOG(WARNING) << "Sensor with id \"" << time_sensor_name << "\" does not exists. Synchronizer was not created.";
      return nullptr;
    }

    return std::make_unique<MasterSensorSynchronizer>(sensor->id());
  } else if (time_type == "no_synchronization") {
    return std::make_unique<NoSynchronization>();
  } else {
    LOG(WARNING) << "Undefined time type";
    return nullptr;
  }
}
}  // namespace synchronizer
}  // namespace dsopp
