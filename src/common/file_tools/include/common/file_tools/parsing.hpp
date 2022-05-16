#ifndef DSOPP_COMMON_FILE_TOOLS_PARSING_HPP_
#define DSOPP_COMMON_FILE_TOOLS_PARSING_HPP_

#include <string>
#include <vector>

#include "common/settings.hpp"
#include "common/time/time.hpp"
#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"

namespace dsopp::common::file_tools {

/**
 * Interprets a Precision value in a string ``str``.
 *
 * @param str string to convert
 * @return string converted to Precision type
 */
Precision stringToPrecision(const std::string &str);

/**
 * Interprets a time value in a string ``str``.
 *
 * @param str string to convert
 * @return string converted to time type
 */
dsopp::time stringToTime(const std::string &str);

/**
 * works similar to python ``split``.
 * Splits ``line`` into N strings separated by ``separator``
 *
 * @param line string to be splitted
 * @param separator separator
 * @return vector of splitted components
 */
std::vector<std::string> splitLine(const std::string &line, char separator);

/**
 * Gets pose from TXT file ``stream``
 *
 * @param stream TXT file
 * @return pose
 */
energy::motion::SE3<Precision> getPose(std::ifstream &stream);

}  // namespace dsopp::common::file_tools

#endif  // DSOPP_COMMON_FILE_TOOLS_PARSING_HPP_
