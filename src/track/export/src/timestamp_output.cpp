#include "track/export/timestamp_output.hpp"

#include <iomanip>
#include <iostream>

namespace dsopp {
namespace test_tools {
std::string timestampToString(const time &timestamp) {
  const int kNano = 9;
  auto seconds = duration_cast<std::chrono::seconds>(timestamp.time_since_epoch()).count();
  auto nanoseconds =
      (timestamp.time_since_epoch() - duration_cast<std::chrono::seconds>(timestamp.time_since_epoch())).count();
  std::stringstream ts;
  ts << seconds << "." << std::setw(kNano) << std::setfill('0') << nanoseconds;
  return ts.str();
}
}  // namespace test_tools
}  // namespace dsopp
