#include "common/fabric_tools/parameters.hpp"

#include <vector>

namespace dsopp::common::fabric_tools {
bool readSwitcherParameter(const std::map<std::string, std::any> &parameters, const std::string &parameter_name) {
  std::vector<std::string> positive_switch = {"on", "true"};
  std::vector<std::string> negative_switch = {"off", "false"};

  if (parameters.count(parameter_name) != 0) {
    std::string parameter_value = std::any_cast<std::string>(parameters.at(parameter_name));
    for (auto &pattern : positive_switch) {
      if (parameter_value == pattern) return true;
    }
    for (auto &pattern : negative_switch) {
      if (parameter_value == pattern) return false;
    }

    LOG(ERROR) << "Unsupported option in \"" + parameter_name + "\". Expected on/off or true/false";
  } else {
    LOG(WARNING) << "Missing \"" + parameter_name + "\" value. Setting to false";
  }
  return false;
}

}  // namespace dsopp::common::fabric_tools
