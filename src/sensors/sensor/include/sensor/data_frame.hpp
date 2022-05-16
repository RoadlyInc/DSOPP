
#ifndef DSOPP_DATA_FRAME_HPP
#define DSOPP_DATA_FRAME_HPP

#include "common/time/time.hpp"

namespace dsopp {
namespace sensors {

/**
 * \brief DataFrame interface.
 *
 * DataFrame is an object that DataProvider sends to the corresponding sensor.
 */
class DataFrame {
 public:
  /**
   * method to access creation time
   * @return Time when the DataFrame was produced by a sensor
   */
  time timestamp() const { return time_; }

  /**
   * method to access DataFrame id
   * @return data frame id
   */
  int id() const { return frame_id_; }

 protected:
  /**
   * creates DataFrame with a given time
   * @param time time of DataFrame creation
   * @param frame_id frame id
   */
  explicit DataFrame(const time time, const int frame_id) : time_(time), frame_id_(frame_id) {}

 private:
  /** Time when the DataFrame was produced by a sensor */
  const time time_;
  /** Unique frame number. */
  const int frame_id_;
};
}  // namespace sensors
}  // namespace dsopp
#endif  // DSOPP_DATA_FRAME_HPP
