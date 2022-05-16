#include "common/file_tools/parsing.hpp"

#include <fstream>
#include <sstream>

namespace dsopp::common::file_tools {

Precision stringToPrecision(const std::string &str) { return static_cast<Precision>(std::stod(str)); }

dsopp::time stringToTime(const std::string &str) {
  const size_t kNano = 9;
  std::vector<std::string> splitted_timestamp = common::file_tools::splitLine(str, '.');
  uint64_t timestamp;
  if (splitted_timestamp.size() > 1) {
    size_t deficiency = kNano - std::min(kNano, splitted_timestamp[1].size());
    timestamp =
        std::stoul(splitted_timestamp[0] + splitted_timestamp[1].substr(0, kNano) + std::string(deficiency, '0'));
  } else {
    timestamp = std::stoul(splitted_timestamp[0] + std::string(kNano, '0'));
  }

  return dsopp::time(std::chrono::nanoseconds(timestamp));
}

std::vector<std::string> splitLine(const std::string &line, char separator) {
  std::vector<std::string> tokens;

  std::stringstream string_stream(line);

  std::string token;
  while (std::getline(string_stream, token, separator)) {
    tokens.push_back(token);
  }

  return tokens;
}

energy::motion::SE3<Precision> getPose(std::ifstream &stream) {
  std::string line;
  std::getline(stream, line);
  const auto tokens = splitLine(line, ' ');
  return energy::motion::SE3<Precision>(
      Eigen::Quaternion<Precision>(stringToPrecision(tokens[6]), stringToPrecision(tokens[3]),
                                   stringToPrecision(tokens[4]), stringToPrecision(tokens[5])),
      energy::motion::SE3<Precision>::Point(stringToPrecision(tokens[0]), stringToPrecision(tokens[1]),
                                            stringToPrecision(tokens[2])));
}

}  // namespace dsopp::common::file_tools
