#ifndef DSOPP_READ_TRACKS_HPP
#define DSOPP_READ_TRACKS_HPP

#include <string>
#include <vector>

#include "common/settings.hpp"
#include "track/track.hpp"
namespace dsopp::application::tools {
/**
 * loads tracks from proto file
 * @param paths vector of proto files
 * @return vector of loaded tracks
 */
std::vector<std::unique_ptr<track::Track<energy::motion::SE3<Precision>>>> readTracks(std::vector<std::string> paths);
}  // namespace dsopp::application::tools

#endif  // DSOPP_READ_TRACKS_HPP
