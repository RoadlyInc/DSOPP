#include "sensors/sensors.hpp"

#include <glog/logging.h>

#include "sensor/sensor.hpp"
#include "sensors/camera/camera.hpp"

namespace dsopp {
namespace sensors {
namespace {
template <class SensorType>
SensorType *getSensorByName(const std::string &name, const std::vector<std::unique_ptr<SensorType>> &sensors) {
  auto sensor_ptr = std::find_if(sensors.begin(), sensors.end(), [&](const auto &s) { return s->name() == name; });
  if (sensor_ptr != sensors.end()) {
    return sensor_ptr->get();
  }
  return nullptr;
}

template <class SensorType>
SensorType *getSensorById(const size_t id, const std::vector<std::unique_ptr<SensorType>> &sensors) {
  auto sensor_ptr = std::find_if(sensors.begin(), sensors.end(), [&](const auto &s) { return s->id() == id; });
  if (sensor_ptr != sensors.end()) {
    return sensor_ptr->get();
  }
  return nullptr;
}

template <class SensorType>
bool sensorValidToAdd(const SensorType &sensor, const std::vector<std::unique_ptr<SensorType>> &sensors,
                      size_t number_of_sensors) {
  const std::string &name = sensor.name();
  if (getSensorByName(name, sensors)) {
    LOG(ERROR) << "Sensor with name \"" << name << "\" already exists.";
    return false;
  }
  if (number_of_sensors != sensor.id()) {
    LOG(ERROR) << "Sensors should be added in order of their identifiers";
    return false;
  }

  return true;
}
}  // namespace

void Sensors::addCamera(std::unique_ptr<Camera> &&camera) {
  if (sensorValidToAdd(*camera, cameras_, sensorsSize())) {
    cameras_.push_back(std::move(camera));
    cameras_references_.emplace_back(cameras_.back().get());
  }
}

Camera *Sensors::getCamera(const size_t id) const { return getSensorById(id, cameras_); }

Camera *Sensors::getCamera(const std::string &name) const { return getSensorByName(name, cameras_); }

Sensor *Sensors::get(const size_t id) const {
  auto camera_candidate = getSensorById(id, cameras_);
  if (camera_candidate) {
    return camera_candidate;
  }

  return nullptr;
}

Sensor *Sensors::get(const std::string &name) const {
  auto camera_candidate = getSensorByName(name, cameras_);
  if (camera_candidate) {
    return camera_candidate;
  }
  return nullptr;
}

const std::deque<const Camera *> &Sensors::cameras() const { return cameras_references_; }

size_t Sensors::sensorsSize() const { return cameras_.size(); }

Sensors::~Sensors() = default;
}  // namespace sensors
}  // namespace dsopp
