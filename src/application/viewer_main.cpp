#include <string>

#include <glog/logging.h>

#include "agent/agent.hpp"
#include "application/tools/read_tracks.hpp"
#include "common/settings.hpp"

#include "output_interfaces/track_output_interface.hpp"
#include "sensors/sensors.hpp"
#include "track/track.hpp"
#include "visualizer/visualizer.hpp"

int main(int argc, char *argv[]) {
  using Motion = dsopp::energy::motion::SE3<dsopp::Precision>;
  google::InitGoogleLogging(argv[0]);
  if (argc < 2) {
    std::cout << "Usage " << argv[0] << " <path to file> <path to file> ..." << std::endl;
    return 0;
  }

  auto visualizer = std::make_unique<dsopp::output::Visualizer>(1920, 1080);
  auto tracks_storage = dsopp::application::tools::readTracks({argv + 1, argv + argc});

  std::vector<dsopp::track::Track<dsopp::energy::motion::SE3<dsopp::Precision>> *> tracks;
  std::transform(tracks_storage.begin(), tracks_storage.end(), back_inserter(tracks),
                 [](const auto &track) { return track.get(); });

  for (const auto &track : tracks) {
    if (track != nullptr) {
      auto *visualizer_oi = visualizer->createTrackOutputInterface<dsopp::track::OdometryTrack, Motion>();
      visualizer_oi->finish(*track);
    }
  }
  visualizer->init();
  while (visualizer->running()) {
    visualizer->render();
  }
}
