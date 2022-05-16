#include "dsopp/dsopp.hpp"

#include <string>
#include <thread>

#include <gflags/gflags.h>
#include <glog/logging.h>
#define TBB_PREVIEW_GLOBAL_CONTROL true
#include <tbb/global_control.h>
#include <opencv2/opencv.hpp>

#include "agent/agent.hpp"
#include "common/settings.hpp"
#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"

#include "output/persistent/protobuf_exporter.hpp"
#include "output_interfaces/image_output_interface.hpp"
#include "output_interfaces/text_output_interface.hpp"
#include "output_interfaces/track_output_interface.hpp"
#include "sensors/sensors.hpp"
#if VISUALIZATION
#include "visualizer/visualizer.hpp"
#endif

DEFINE_string(config_file_path, MONO_CONFIG_FILE, "path to config file");
DEFINE_string(output_file_path, "track.bin", "path to output file");
DEFINE_bool(visualization, true, "flag to enable visualization");
DEFINE_bool(deterministic, false, "flag to turn off parallelism");
DEFINE_bool(refine_calibration, false, "flag to enable camera calibration refinement");
DEFINE_uint32(start_frame, 0, "start frame to optimize in camera calibration refinement");
DEFINE_uint32(frames_number, 80, "number of frames to optimize in camera calibration refinement");

#ifdef ROLLING_SHUTTER_BUILD
// using Motion = dsopp::energy::motion::UniformRollingShutter<dsopp::Precision>;
#else
using Motion = dsopp::energy::motion::SE3<dsopp::Precision>;
#endif

std::map<std::string, std::string> parseConfigArgs(int argc, char *argv[]) {
  std::map<std::string, std::string> config_args;
  for (int i = 1; i < argc; ++i) {
    std::string key_value = argv[i];
    std::string delimiter = "--config.";
    auto config_idx = key_value.find(delimiter);
    if (config_idx != std::string::npos) {
      key_value = key_value.substr(config_idx + delimiter.size());
      delimiter = "=";
      auto key_idx = key_value.find(delimiter);
      if (key_idx != std::string::npos) {
        config_args[key_value.substr(0, key_idx)] = key_value.substr(key_idx + delimiter.size());
      }
    }
  }
  return config_args;
}

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  gflags::SetVersionString(GIT_COMMIT_HASH);
  std::string usage = "Usage:\n";
  usage += argv[0];
  usage +=
      "[--helpshort] [--version].\nYou can overwrite the "
      "configurations received from the file. To do this, add a "
      "flag that starts with --config, separating each field with a dot. For example "
      "--config.tracker.keyframe_strategy.factor=50.\nIf you want to change the field in the sequence, refer to it by "
      "order "
      "number (started from 0). For "
      "example --config.sensors.0.provider.start_frame=3000 ";
  gflags::SetUsageMessage(usage);
  gflags::AllowCommandLineReparsing();
  gflags::ParseCommandLineFlags(&argc, &argv, false);

  auto config_args = parseConfigArgs(argc, argv);

  const size_t kSaveStride = 150;

  std::unique_ptr<dsopp::DSOPP<Motion>> dsopp;
  dsopp = dsopp::DSOPP<Motion>::loadFromYaml(FLAGS_config_file_path, config_args);

  if (!dsopp) {
    LOG(ERROR) << "DSOPP instance was not created";
  }
  auto saver = std::make_unique<dsopp::output::ProtobufExporter<dsopp::track::ActiveOdometryTrack, Motion>>(
      FLAGS_output_file_path, kSaveStride);

  dsopp->addTrackOutputInterface(*saver);

#if VISUALIZATION
  std::cout << "test" << std::endl;
  std::thread runthread;
  auto visualizer = std::make_unique<dsopp::output::Visualizer>(1920, 1080);
  if (FLAGS_visualization) {
    const size_t kDebugImageWidth = 1280 / 3;
    const size_t kDebugImageHeight = 720 / 3;
    dsopp->addTrackOutputInterface(
        *visualizer->createTrackOutputInterface<dsopp::track::ActiveOdometryTrack, Motion>());
    dsopp->addDebugCameraOutputInterfaces(
        visualizer
            ->createCameraOutputInterfaces(dsopp->agent().sensors().cameras(), kDebugImageWidth, kDebugImageHeight)
            .at(0));
    dsopp->addTextOutputInterface("status", visualizer->createTextOutputInterface("status"));

    runthread = std::thread([&]() {
      visualizer->init();

      while (visualizer->running()) {
        visualizer->render();
      }
    });
  }
#endif

  const uint32_t kMaxNumberOfThreads = FLAGS_deterministic ? 2u : 8u;

  const size_t number_of_threads = std::clamp(std::thread::hardware_concurrency(), 1u, kMaxNumberOfThreads);
  tbb::global_control tbb_thread_count(tbb::global_control::max_allowed_parallelism, number_of_threads - 1);

  dsopp->run(number_of_threads - 1);

#if VISUALIZATION
  if (runthread.joinable()) {
    runthread.join();
  }
#endif
}
