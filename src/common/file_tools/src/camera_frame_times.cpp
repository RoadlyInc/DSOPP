#include "common/file_tools/camera_frame_times.hpp"

#include <glog/logging.h>
#include <filesystem>
#include <fstream>

#include "common/file_tools/parsing.hpp"

namespace dsopp::common::file_tools {

void readTimes(const std::string &timestamps_file, std::map<uint64_t, CameraFrameTimes> &times) {
  auto stream = std::ifstream(timestamps_file);
  if (!stream.is_open()) {
    LOG(WARNING) << "File " << timestamps_file << " does not exist!";
    return;
  }
  std::string line;
  while (std::getline(stream, line)) {
    const auto tokens = splitLine(line, ' ');
    uint64_t frame_id = std::stoull(tokens[0]);
    std::string ts_string = tokens[1];
    const uint64_t timestamp =
        static_cast<uint64_t>(common::file_tools::stringToTime(ts_string).time_since_epoch().count());
    Precision exposure_time = 1;
    if (tokens.size() > 2) {
      exposure_time = stringToPrecision(tokens[2]);
    }
    times.insert({frame_id, {timestamp, exposure_time}});
  }
  stream.close();
}

}  // namespace dsopp::common::file_tools
