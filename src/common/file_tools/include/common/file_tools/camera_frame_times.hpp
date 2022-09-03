#ifndef DSOPP_COMMON_FILE_TOOLS_CAMERA_FRAME_TIMES_HPP_
#define DSOPP_COMMON_FILE_TOOLS_CAMERA_FRAME_TIMES_HPP_

#include <map>
#include <string>

#include "common/settings.hpp"

namespace dsopp::common::file_tools {

/** Camera frame times */
struct CameraFrameTimes {
  /** timestamp */
  const uint64_t timestamp;
  /** exposure time */
  const Precision exposure_time;
};

/**
 * function to read all times of frames
 *
 * @param timestamps_file path to the file with timestamps
 * @param[out] times all times of frames
 */
void readTimes(const std::string &timestamps_file, std::map<uint64_t, CameraFrameTimes> &times);

}  // namespace dsopp::common::file_tools

#endif  // DSOPP_COMMON_FILE_TOOLS_CAMERA_FRAME_TIMES_HPP_
