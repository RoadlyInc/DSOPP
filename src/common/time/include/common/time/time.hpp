#ifndef DSOPP_SRC_TIME_HPP_
#define DSOPP_SRC_TIME_HPP_

#include <chrono>
#include <ostream>

namespace dsopp {
/** shortcut for timestamp  */
using time = std::chrono::time_point<std::chrono::high_resolution_clock>;
std::ostream& operator<<(std::ostream& os, const time&);
}  // namespace dsopp

#endif  // DSOPP_SRC_TIME_HPP_
