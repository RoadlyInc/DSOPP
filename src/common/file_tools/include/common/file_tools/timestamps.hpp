#ifndef DSOPP_SENSORS_CAMERA_PROVIDER_TIMESTAMPS_HPP_
#define DSOPP_SENSORS_CAMERA_PROVIDER_TIMESTAMPS_HPP_

#include <map>
#include <string>

namespace dsopp::sensors::providers {

/**
 * function to read all timestamps of frames
 *
 * @param timestamps_file path to the file with timestamps
 * @param[out] timestamps all timestamps of frames
 */
void readTimestamps(const std::string &timestamps_file, std::map<uint64_t, uint64_t> &timestamps);

}  // namespace dsopp::sensors::providers

#endif  // DSOPP_SENSORS_CAMERA_PROVIDER_TIMESTAMPS_HPP_
