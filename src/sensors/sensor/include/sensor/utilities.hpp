#ifndef DSOPP_UTILITIES_HPP
#define DSOPP_UTILITIES_HPP

#include <memory>

#include "common/time/time.hpp"

namespace dsopp {
namespace sensors {
namespace utilities {
template <class SensorDataFrame, class SensorProvider>
void fillNextFrame(std::unique_ptr<SensorDataFrame> &next_frame, int &next_frame_id, time &next_frame_time,
                   SensorProvider &provider) {
  if (!next_frame) {
    next_frame = provider.nextFrame();
    if (next_frame) {
      next_frame_id = next_frame->id();
      next_frame_time = next_frame->timestamp();
    }
  }
}
}  // namespace utilities
}  // namespace sensors
}  // namespace dsopp

#endif  // DSOPP_UTILITIES_HPP
