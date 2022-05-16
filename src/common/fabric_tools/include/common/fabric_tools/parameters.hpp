#ifndef DSOPP_COMMON_FABRIC_TOOLS_PARAMETERS_HPP
#define DSOPP_COMMON_FABRIC_TOOLS_PARAMETERS_HPP

#include <any>
#include <map>
#include <sstream>
#include <string>

#include <glog/logging.h>

namespace dsopp::common::fabric_tools {
/**
 * checks if parameter is in config and fills it
 *
 * @tparam T value type
 * @param[out] value parameter to be filled or not
 * @param parameters parameters
 * @param parameter_name name of parameter in config
 */
template <typename T>
bool readParameter(T &value, const std::map<std::string, std::any> &parameters, const std::string &parameter_name,
                   bool message = true) {
  if (parameters.count(parameter_name) != 0) {
    std::string str_value = std::any_cast<std::string>(parameters.at(parameter_name));
    std::stringstream streamed_value(str_value);
    streamed_value >> value;
    return true;
  }
  if (message) {
    LOG(WARNING) << "Missing \"" + parameter_name + "\" value. Setting to " << value;
  }
  return false;
}

/**
 * reads switcher paramter and retruns bool
 * ``true`` if value is "on"\"true"
 * ``false`` if value is "offf"\"false" or not found
 *
 * @param pararmters paramters
 * @param parameter_name parameter name to find
 */
bool readSwitcherParameter(const std::map<std::string, std::any> &parameters, const std::string &parameter_name);

}  // namespace dsopp::common::fabric_tools

#endif  // DSOPP_COMMON_FABRIC_TOOLS_PARAMETERS_HPP
