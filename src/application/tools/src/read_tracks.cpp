#include "application/tools/read_tracks.hpp"

#include <fstream>

#include "storage/track_storage.hpp"

namespace dsopp::application::tools {
using Motion = energy::motion::SE3<Precision>;

std::vector<std::unique_ptr<track::Track<Motion>>> readTracks(std::vector<std::string> paths) {
  std::vector<std::unique_ptr<track::Track<Motion>>> tracks;
  for (const auto &track_file : paths) {
    std::ifstream stream(track_file, std::ios::binary);
    if (!stream.is_open()) {
      std::cout << "Can't open file " << track_file << std::endl;
      tracks.push_back(nullptr);
      continue;
    }

    track::storage::Track track_storage;
    if (!track_storage.read(stream)) {
      std::cout << "Can't parse file " << track_file << std::endl;
      tracks.push_back(nullptr);
      continue;
    }

    tracks.push_back(std::make_unique<track::Track<Motion>>(track_storage));
    stream.close();
  }
  return tracks;
}
}  // namespace dsopp::application::tools
