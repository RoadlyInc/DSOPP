#ifndef DSOPP_SRC_SYNCHRONIZER_MASTER_SENSOR_SYNCHRONIZER_HPP_
#define DSOPP_SRC_SYNCHRONIZER_MASTER_SENSOR_SYNCHRONIZER_HPP_

#include "common/time/time.hpp"
#include "synchronizer/synchronizer.hpp"

namespace dsopp {
namespace synchronizer {
/**
 * \brief Synchronizer based on one of the sensors.
 *
 * This type of synchronizer based on the inner time of one of the sensors.
 * For example, it can be a camera.
 */
class MasterSensorSynchronizer : public Synchronizer {
 public:
  /**
   * creates Synchronizer based on the sensor with the given id
   * @param sensor_id unique identifier for sensor
   */
  explicit MasterSensorSynchronizer(size_t sensor_id);
  /**
   * return inner time of the Synchronizer
   * @return inner time of the Synchronizer
   */
  time currentTime() override;
  /**
   * method to get an array of features frames from all sensors corresponding to the inner time.
   * @param sensors to be synchronized
   * @return an array of features frames from all sensors corresponding to the inner time
   */
  std::unique_ptr<sensors::SynchronizedFrame> sync(sensors::Sensors &sensors) override;
  ~MasterSensorSynchronizer() override = default;

 private:
  /** unique identifier for the sensor, on the basis of which the synchronizer operates*/
  const size_t sensor_id_;
};
}  // namespace synchronizer
}  // namespace dsopp

#endif  // DSOPP_SRC_SYNCHRONIZER_MASTER_SENSOR_SYNCHRONIZER_HPP_
