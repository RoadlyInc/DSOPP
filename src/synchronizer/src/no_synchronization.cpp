#include "synchronizer/no_synchronization.hpp"

#include <iostream>
#include <memory>

#include "sensor/sensor.hpp"
#include "sensor/synchronized_frame.hpp"
#include "sensors/sensors.hpp"

namespace dsopp {
namespace synchronizer {
NoSynchronization::NoSynchronization() : current_frame_id_(0) {}

time NoSynchronization::currentTime() { return current_time_; }

std::unique_ptr<sensors::SynchronizedFrame> NoSynchronization::sync(sensors::Sensors &sensors) {
  auto min_time = time::max();
  for (size_t sensor_id = 0; sensor_id < sensors.sensorsSize(); ++sensor_id) {
    auto *sensor = sensors.get(sensor_id);
    if (!sensor->empty()) {
      min_time = min(min_time, sensor->nextFrameTime());
    }
  }

  if (min_time == time::max()) {
    return nullptr;
  }

  current_time_ = min_time;
  auto frame = std::make_unique<sensors::SynchronizedFrame>(current_frame_id_++, current_time_);
  for (size_t sensor_id = 0; sensor_id < sensors.sensorsSize(); ++sensor_id) {
    auto *sensor = sensors.get(sensor_id);
    if (sensor->nextFrameTime() == min_time) {
      sensor->processNextDataFrame(*frame);
    }
  }
  return frame;
}
}  // namespace synchronizer
}  // namespace dsopp
