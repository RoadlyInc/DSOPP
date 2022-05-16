#ifndef DSOPP_TIMESTAMP_OUTPUT_HPP
#define DSOPP_TIMESTAMP_OUTPUT_HPP

#include <fstream>

#include "common/time/time.hpp"

namespace dsopp {
namespace test_tools {
std::string timestampToString(const time &timestamp);

}  // namespace test_tools
}  // namespace dsopp

#endif  // DSOPP_TIMESTAMP_OUTPUT_HPP
