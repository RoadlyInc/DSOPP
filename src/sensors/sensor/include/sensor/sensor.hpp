
#ifndef DSOPP_SENSOR_HPP
#define DSOPP_SENSOR_HPP

#include <memory>
#include <string>
#include <vector>

#include "common/time/time.hpp"

namespace dsopp {
namespace sensors {
class SynchronizedFrame;
/**
 * \brief Sensor interface.
 *
 * Every entity that supplies data to the system is called a Sensor and should implement this interface.
 */
class Sensor {
 public:
  /**
   * Process next data frame from provider
   * @param frame  frame to be filled by a sensor
   * @return false if sensor stream ended and true otherwise
   */
  virtual bool processNextDataFrame(sensors::SynchronizedFrame &frame) = 0;

  /**
   * method to access identifier (name) of the Sensor
   * @return sensor identifier (name)
   */
  const std::string &name() const { return name_; }
  /**
   * method to access identifier of the Sensor
   * @return sensor identifier
   */
  size_t id() const { return id_; }
  /**
   * method to access index of next frame
   * @return index of next frame
   */
  virtual int nextFrameId() = 0;
  /**
   * method to access time of next frame
   * @return time of next frame
   */
  virtual time nextFrameTime() = 0;
  /**
   * method to know if sensor stream ended
   * @return true if sensor stream ended and false otherwise
   */
  virtual bool empty() = 0;

  virtual ~Sensor() = default;

 protected:
  /**
   * creates Sensor with a given id
   * @param name unique identifier of sensor
   * @param id unique identifier of sensor
   */
  explicit Sensor(const std::string name, size_t id) : name_(name), id_(id) {}
  /** unique identifier (name) of sensor */
  std::string name_;
  /** unique identifier of sensor */
  size_t id_;
};
}  // namespace sensors
}  // namespace dsopp

#endif  // DSOPP_SENSOR_HPP
