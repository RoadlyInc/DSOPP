#ifndef DSOPP_SRC_SYNCHRONIZER_NO_SYNCHRONIZATION_HPP_
#define DSOPP_SRC_SYNCHRONIZER_NO_SYNCHRONIZATION_HPP_

#include "common/time/time.hpp"
#include "synchronizer/synchronizer.hpp"

namespace dsopp {
namespace synchronizer {
/**
 * \brief Synchronizer based on sensors with minimum inner time.
 *
 * This type of synchronizer based on the minimum inner time of the sensors.
 */
class NoSynchronization : public Synchronizer {
 public:
  /**
   * creates Synchronizer based on sensors with minimum inner time
   */
  explicit NoSynchronization();
  /**
   * return inner time of the Synchronizer
   * @return inner time of the Synchronizer
   */
  time currentTime() override;
  /**
   * method to get an array of features frames from sensors with minimum inner time.
   * @param sensors to be synchronized
   * @return an array of features frames from sensors with minimum inner time
   */
  std::unique_ptr<sensors::SynchronizedFrame> sync(sensors::Sensors &sensors) override;
  ~NoSynchronization() override = default;

 private:
  /** inner time of the Synchronizer*/
  time current_time_;
  /** current frame index */
  int current_frame_id_;
};
}  // namespace synchronizer
}  // namespace dsopp

#endif  // DSOPP_SRC_SYNCHRONIZER_NO_SYNCHRONIZATION_HPP_
