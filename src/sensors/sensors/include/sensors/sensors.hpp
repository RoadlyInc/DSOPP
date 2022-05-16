#ifndef DSOPP_SRC_SENSORS_SENSORS_HPP_
#define DSOPP_SRC_SENSORS_SENSORS_HPP_

#include <deque>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace dsopp {
namespace sensors {
class Sensor;
class Camera;
class IMU;
class Gnss;
/**
 * \brief Container for sensors
 *
 * All sensors of the Agent have to be stored in this container.
 */
class Sensors {
 public:
  /**
   * method to add new camera to container
   * @param camera new camera to add
   */
  void addCamera(std::unique_ptr<Camera> &&camera);
  /**
   * method to get the camera by name
   * @param name unique identifier for camera
   * @return pointer to the camera with name if exists and nullptr otherwise
   */
  Camera *getCamera(const std::string &name) const;
  /**
   * method to get the camera by id
   * @param id unique identifier for camera
   * @return reference to the camera with id if exists and nullptr otherwise
   */
  Camera *getCamera(const size_t id) const;
  /**
   * method to get the sensor by id
   * @param id unique identifier for sensor
   * @return reference to the sensor with id if exists and nullptr otherwise
   */
  Sensor *get(const size_t id) const;
  /**
   * method to get the sensor by name
   * @param name unique identifier for sensor
   * @return reference to the sensor with name if exists and nullptr otherwise
   */
  Sensor *get(const std::string &name) const;
  /**
   * method to get references to all cameras.
   * @return vector of references to all cameras
   */
  const std::deque<const Camera *> &cameras() const;
  /**
   * @return amount of all sensors
   */
  size_t sensorsSize() const;
  ~Sensors();

 private:
  /** Container containing cameras of the Agent */
  std::vector<std::unique_ptr<Camera>> cameras_;
  /** Container containing raw pointers to cameras to provide access */
  std::deque<const Camera *> cameras_references_;
  /** Container containing raw pointers to gnsss to provide access */
  std::deque<const Gnss *> gnsss_references_;
  /** Container containing raw pointers to imus to provide access */
  std::deque<const IMU *> imus_references_;
};
}  // namespace sensors
}  // namespace dsopp

#endif  // DSOPP_SRC_SENSORS_SENSORS_HPP_
