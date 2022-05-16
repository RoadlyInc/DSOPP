#include "common/time/time.hpp"

#include <iomanip>
#include <string>

namespace {
std::string convertTimePointToString(const dsopp::time& time_point) {
  std::time_t time = std::chrono::system_clock::to_time_t(time_point);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_point.time_since_epoch()) -
            std::chrono::duration_cast<std::chrono::seconds>(time_point.time_since_epoch());
  std::stringstream ss;
  ss << std::put_time(std::localtime(&time), "%H h %M m %S s ");
  return ss.str() + std::to_string(ms.count()) + std::string(" ms");
}
}  // namespace

std::ostream& dsopp::operator<<(std::ostream& os, const dsopp::time& time_point) {
  os << convertTimePointToString(time_point);
  return os;
}
