
#ifndef DSOPP_SYNCHRONIZER_HPP
#define DSOPP_SYNCHRONIZER_HPP

#include <memory>
#include <vector>

#include "common/time/time.hpp"

namespace dsopp {
namespace sensors {
class Sensors;
class SynchronizedFrame;
}  // namespace sensors
namespace synchronizer {
/** \brief Synchronizer interface
 *
 * Synchronizer is a class to manage system time for sensor synchronization.
 * There is only one synchronizer in the system.
 */
class Synchronizer {
 public:
  /**
   * method to get the current time of the system
   *
   * @return current time of the system
   */
  virtual time currentTime() = 0;
  /**
   * method to get an array of features frames from all sensors corresponding to the inner time.
   *
   * @param sensors to be synchronized
   * @return an array of features frames from all sensors corresponding to the inner time
   */
  virtual std::unique_ptr<sensors::SynchronizedFrame> sync(sensors::Sensors &sensors) = 0;
  virtual ~Synchronizer() = default;
};
}  // namespace synchronizer
}  // namespace dsopp
#endif  // DSOPP_SYNCHRONIZER_HPP
