#include <glog/logging.h>
#include <fstream>
#include <string>

#include "common/settings.hpp"

#include "storage/track_storage.hpp"
#include "track/export/track2tum_exporter.hpp"
#include "track/track.hpp"

template <dsopp::energy::motion::Motion Motion>
void exportPoses(const dsopp::track::storage::Track &track_storage, const std::string &trajectory_type,
                 const std::string &output_filename) {
  auto track = std::make_unique<dsopp::track::Track<Motion>>(track_storage);

  if (trajectory_type == "odometry") {
    dsopp::test_tools::track2TumExporter(track->odometryTrack(), output_filename);
  } else if (trajectory_type == "ecef_poses") {
    LOG(FATAL) << "contact Roadly INC for this functionality";
  }
}

int main(int argc, char *argv[]) {
  if (argc != 3 && argc != 4) {
    std::cout << "Usage " << argv[0] << " <trajectory type>=odometry|ecef_poses <path to file> [<output filename>]"
              << std::endl;
    return 1;
  }
  std::string trajectory_type = argv[1];
  std::string track_file = argv[2];
  std::string output_filename;
  if (trajectory_type == "odometry") {
    output_filename = "output_track_positions.tum";
  } else if (trajectory_type == "ecef_poses") {
    output_filename = "output_ecef_poses.enu";
  } else {
    std::cout << "Wrong trajectory_type " << trajectory_type << std::endl;
    return 1;
  }
  if (argc == 4) {
    output_filename = argv[3];
  }
  std::ifstream stream(track_file, std::ios::binary);
  if (!stream.is_open()) {
    std::cout << "Can't open file " << track_file << std::endl;
    return 1;
  }

  dsopp::track::storage::Track track_storage;
  if (!track_storage.read(stream)) {
    std::cout << "Can't parse file " << track_file << std::endl;
    return 1;
  }

  if (track_storage.odometryTrack().frames.size() == 0) {
    std::cout << "Empty frames in track file. Terminating..." << std::endl;
    return 1;
  }
  stream.close();

  bool SE3_motion = track_storage.odometryTrack().frames[0].t_world_agent_size() ==
                    dsopp::energy::motion::SE3<dsopp::Precision>::num_parameters;
  if (SE3_motion)
    exportPoses<dsopp::energy::motion::SE3<dsopp::Precision>>(track_storage, trajectory_type, output_filename);
}
