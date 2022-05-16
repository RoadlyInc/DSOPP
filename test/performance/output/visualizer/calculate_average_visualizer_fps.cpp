#include <memory>

#include "output_interfaces/track_output_interface.hpp"
#include "visualizer/visualizer.hpp"

#include "agent/agent.hpp"
#include "agent/agent_settings.hpp"
#include "common/settings.hpp"
#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"
#include "sensors/sensors.hpp"
#include "test/tools/random_track.hpp"
#include "track/odometry_track.hpp"
#include "track/track.hpp"

using namespace dsopp;
using namespace output;

int main() {
  using SE3 = dsopp::energy::motion::SE3<Precision>;

  const size_t kTracksNumber = 10;
  const auto duration = std::chrono::seconds(20);
  auto visualizer = std::make_unique<Visualizer>(1920, 1080);
  auto *visualizer_FPS = visualizer->createTrackOutputInterface<dsopp::track::OdometryTrack, SE3>();

  std::vector<std::unique_ptr<track::Track<SE3>>> tracks;
  for (size_t track_index = 0; track_index < kTracksNumber; ++track_index) {
    auto track = test_tools::randomTrack(static_cast<int>(track_index * 2));
    tracks.push_back(std::make_unique<dsopp::track::Track<SE3>>(nullptr, std::move(track)));
    visualizer_FPS->notify(*tracks[track_index]);
  }
  visualizer->init();
  const auto start = std::chrono::high_resolution_clock::now();
  size_t frames_count = 0;
  while (visualizer->running() && std::chrono::high_resolution_clock::now() - start <= duration) {
    visualizer->render();
    ++frames_count;
  }
  std::cout << "average visualizer FPS = " << double(frames_count) / double(duration.count()) << '\n';
  return 0;
}
