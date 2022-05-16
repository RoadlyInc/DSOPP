#include "common/file_tools/timestamps.hpp"

#include <glog/logging.h>
#include <filesystem>
#include <fstream>

#include "common/file_tools/parsing.hpp"

namespace dsopp::sensors::providers {

void readTimestamps(const std::string &timestamps_file, std::map<uint64_t, uint64_t> &timestamps) {
  auto stream = std::ifstream(timestamps_file);
  if (!stream.is_open()) {
    LOG(WARNING) << "File " << timestamps_file << " does not exist!";
    return;
  }
  std::string line;
  while (std::getline(stream, line)) {
    std::istringstream line_stream(line);
    uint64_t frame_id;
    std::string ts_string;
    line_stream >> frame_id >> ts_string;
    timestamps[frame_id] =
        static_cast<uint64_t>(common::file_tools::stringToTime(ts_string).time_since_epoch().count());
  }
  stream.close();
}

}  // namespace dsopp::sensors::providers
