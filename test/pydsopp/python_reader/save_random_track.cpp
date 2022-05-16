#include <fstream>

#include "agent/agent_settings.hpp"
#include "common/settings.hpp"
#include "sanity_checker/sanity_checker.hpp"
#include "storage/track_storage.hpp"
#include "test/tools/random_track.hpp"
#include "track/odometry_track.hpp"
#include "track/track.hpp"

namespace dsopp::track {
namespace {
void save(const std::string& filename = "track.bin") {
  auto track = test_tools::track();
  auto track_storage = track->serialize();

  std::ofstream ostream(filename, std::ios::binary);
  track_storage.save(ostream);
  ostream.close();
}
}  // namespace
}  // namespace dsopp::track

int main(int argc, char** argv) {
  if (argc == 2) {
    dsopp::track::save(argv[1]);
  } else
    dsopp::track::save();
  return 0;
}
