#include "synchronizer/master_sensor_synchronizer.hpp"

#include <memory>

#include "sensor/sensor.hpp"
#include "sensor/synchronized_frame.hpp"
#include "sensors/sensors.hpp"

namespace dsopp {
namespace synchronizer {
MasterSensorSynchronizer::MasterSensorSynchronizer(size_t sensor_id) : sensor_id_(sensor_id) {}
time MasterSensorSynchronizer::currentTime() { return time(std::chrono::milliseconds(0)); }
std::unique_ptr<sensors::SynchronizedFrame> MasterSensorSynchronizer::sync(sensors::Sensors &sensors) {
  auto *sensor = sensors.get(sensor_id_);
  auto frame = std::make_unique<sensors::SynchronizedFrame>(sensor->nextFrameId(), sensor->nextFrameTime());
  if (sensor->processNextDataFrame(*frame)) {
    return frame;
  }
  return nullptr;
}
}  // namespace synchronizer
}  // namespace dsopp
