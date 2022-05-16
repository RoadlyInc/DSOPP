#include <fstream>

#include "common/settings.hpp"
#include "storage/track_storage.hpp"
#include "test/tools/compare_track.hpp"
#include "track/track.hpp"

namespace dsopp::track {
namespace {
void compare(const std::string &filename = "track.bin", const std::string &filename_python = "track_python.bin") {
  using SE3 = energy::motion::SE3<dsopp::Precision>;
  const Precision kEps = 1e-14_p;
  storage::Track track_storage;

  std::ifstream istream(filename, std::ios::binary);
  track_storage.read(istream);
  auto track = Track<SE3>(track_storage);
  istream.close();

  std::ifstream istream_python(filename_python, std::ios::binary);
  track_storage.read(istream_python);
  auto track_python = Track<SE3>(track_storage);
  istream_python.close();

  test_tools::compareTrack(track, track_python, kEps);
}
}  // namespace
}  // namespace dsopp::track

int main(int argc, char **argv) {
  if (argc == 3) {
    dsopp::track::compare(argv[1], argv[2]);
  } else
    dsopp::track::compare();
  return 0;
}
