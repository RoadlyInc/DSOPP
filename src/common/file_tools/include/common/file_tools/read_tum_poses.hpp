#ifndef DSOPP_COMMON_FILE_TOOLS_READ_TUM_FORMAT_HPP_
#define DSOPP_COMMON_FILE_TOOLS_READ_TUM_FORMAT_HPP_

#include <fstream>
#include <map>

#include "common/settings.hpp"
#include "common/time/time.hpp"
#include "energy/motion/motion.hpp"

namespace dsopp::common::file_tools {

/**
 * Reads tum timestamps and poses in tum format
 * @param stream input stream
 * @return map of timestamp - Motion
 */
template <energy::motion::Motion Motion>
std::map<time, Motion> readTumPoses(std::ifstream &stream);

}  // namespace dsopp::common::file_tools

#endif  // DSOPP_COMMON_FILE_TOOLS_READ_TUM_FORMAT_HPP_
