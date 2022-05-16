#include "dsopp/dsopp.hpp"

#include <chrono>
#include <queue>
#include <sstream>
#include <thread>

#include <glog/logging.h>

#include "agent/agent.hpp"
#include "common/settings.hpp"
#include "dsopp/config_loader.hpp"
#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"

#include "feature_based_slam/tracker/tracker.hpp"
#include "output_interfaces/image_output_interface.hpp"
#include "output_interfaces/text_output_interface.hpp"
#include "output_interfaces/track_output_interface.hpp"
#include "sanity_checker/sanity_checker.hpp"
#include "sensor/synchronized_frame.hpp"
#include "sensors/camera/camera.hpp"
#include "sensors/sensors.hpp"
#include "synchronizer/synchronizer.hpp"
#include "track/active_odometry_track.hpp"
#include "track/active_track.hpp"
#include "tracker/tracker.hpp"

namespace dsopp {

namespace {
void setupTextInterfaces(std::map<std::string, output::TextOutputInterface *> &text_interfaces,
                         output::TextOutputInterface *&status_text_interface) {
  if (text_interfaces.contains("status")) {
    status_text_interface = text_interfaces["status"];
  }
}

/**
 * \brief fill status info (frame id and FPS)
 * @param start_time time point form slam first frame
 * @param frame_n current number of processed frames
 * @return status string
 */
std::string statusBarInfo(const std::chrono::time_point<std::chrono::high_resolution_clock> &start_time,
                          size_t frame_n) {
  const size_t kCurrentFpsFramesNumber = 50;
  using TimeQueue = std::queue<std::chrono::time_point<std::chrono::high_resolution_clock>>;
  static TimeQueue current_fps_frames_times(TimeQueue::container_type{start_time});
  static std::queue<size_t> frame_numbers({0});

  const auto stop_time = std::chrono::high_resolution_clock::now();
  current_fps_frames_times.push(stop_time);
  frame_numbers.push(frame_n);
  if (current_fps_frames_times.size() > kCurrentFpsFramesNumber + 1) {
    current_fps_frames_times.pop();
    frame_numbers.pop();
  }

  const Precision current_duration =
      1e-3_p *
      static_cast<Precision>(
          std::chrono::duration_cast<std::chrono::milliseconds>(stop_time - current_fps_frames_times.front()).count());
  const Precision total_duration =
      1e-3_p *
      static_cast<Precision>(std::chrono::duration_cast<std::chrono::milliseconds>(stop_time - start_time).count());

  std::stringstream status;
  status << "Frame N " << frame_n << ", Current FPS : " << std::fixed << std::setprecision(3)
         << static_cast<Precision>(frame_n - frame_numbers.front()) / current_duration << ", Total FPS : " << std::fixed
         << std::setprecision(3) << static_cast<Precision>(frame_n) / total_duration;
  return status.str();
}

}  // namespace

template <energy::motion::Motion Motion>
DSOPP<Motion>::DSOPP(std::unique_ptr<track::ActiveTrack<Motion>> &&track, std::unique_ptr<Agent> &&agent,
                     std::unique_ptr<synchronizer::Synchronizer> &&synchronizer,
                     std::unique_ptr<tracker::Tracker<Motion>> &&tracker,
                     std::unique_ptr<feature_based_slam::tracker::Tracker> &&feature_based_slam_tracker,
                     std::unique_ptr<sanity_checker::SanityChecker<track::ActiveTrack, Motion>> &&sanity_checker)
    : agent_(std::move(agent)),
      synchronizer_(std::move(synchronizer)),
      tracker_(std::move(tracker)),
      feature_based_slam_tracker_(std::move(feature_based_slam_tracker)),
      sanity_checker_(std::move(sanity_checker)),
      track_(std::move(track)) {}

template <energy::motion::Motion Motion>
template <bool TRANSFORM_TO_PINHOLE>
std::unique_ptr<DSOPP<Motion>> DSOPP<Motion>::loadFromYaml(const std::string &file,
                                                           const std::map<std::string, std::string> &config_args) {
  ConfigLoader loader(file);
  return loader.application<Motion, TRANSFORM_TO_PINHOLE>(config_args);
}

template <energy::motion::Motion Motion>
void DSOPP<Motion>::run(const size_t number_of_threads) {
  LOG(INFO) << "Number of threads : " << number_of_threads << '(' << std::thread::hardware_concurrency() << ')';

  output::TextOutputInterface *status_text_interface = nullptr;
  setupTextInterfaces(text_interfaces_, status_text_interface);

  auto start_time = std::chrono::high_resolution_clock::now();
  std::vector<std::unique_ptr<sensors::SynchronizedFrame>> frames_storage;

  for (auto &output_interface : track_output_interfaces_) {
    output_interface.get().notify(*track_);
  }

  const size_t step = 1;
  size_t camera_frame_count = 0;
  size_t acceptable_remainder = 0;

  while (auto frame = synchronizer_->sync(agent_->sensors())) {
    LOG(INFO) << "processing frame " << frame->id();
    auto frame_ptr = frame.get();

    bool is_camera_frame = !frame->cameraFeatures().empty();

    if (is_camera_frame && camera_frame_count++ % step != acceptable_remainder) {
      continue;
    }

    if (!tracker_->isInitialized()) {
      frames_storage.push_back(std::move(frame));
      frame_ptr = frames_storage.back().get();
      feature_based_slam_tracker_->tick(*frame_ptr, number_of_threads);
      if (feature_based_slam_tracker_->initialized()) {
        tracker_->initialize(*feature_based_slam_tracker_, frames_storage, *track_, number_of_threads);
        frame = std::move(frames_storage.back());
        frame_ptr = frame.get();
        frames_storage.clear();
      }
    } else {
      tracker_->tick(*track_, *frame, number_of_threads);
    }
    for (auto &output_interface : track_output_interfaces_) {
      output_interface.get().notify(*track_);
    }

    if (status_text_interface && is_camera_frame) {
      status_text_interface->pushText(statusBarInfo(start_time, camera_frame_count));
    }
  }

  for (auto &output_interface : track_output_interfaces_) {
    output_interface.get().finish(*track_);
  }
}

template <energy::motion::Motion Motion>
void DSOPP<Motion>::addTrackOutputInterface(
    output::TrackOutputInterface<track::ActiveOdometryTrack, Motion> &track_output_interface) {
  track_output_interfaces_.emplace_back(track_output_interface);
}

template <energy::motion::Motion Motion>
void DSOPP<Motion>::addTextOutputInterface(const std::string &name, output::TextOutputInterface *output_interface) {
  text_interfaces_[name] = output_interface;
}

template <energy::motion::Motion Motion>
void DSOPP<Motion>::addDebugCameraOutputInterfaces(const output::CameraOutputInterfaces &output_interfaces) {
  tracker_->setDebugCameraOutputInterface(output_interfaces);
}

template <energy::motion::Motion Motion>
DSOPP<Motion>::~DSOPP() = default;

template <energy::motion::Motion Motion>
const track::ActiveTrack<Motion> &DSOPP<Motion>::track() const {
  return *track_;
}

template <energy::motion::Motion Motion>
const Agent &DSOPP<Motion>::agent() const {
  return *agent_;
}

template class DSOPP<energy::motion::SE3<Precision>>;
template std::unique_ptr<DSOPP<energy::motion::SE3<Precision>>>
DSOPP<energy::motion::SE3<Precision>>::loadFromYaml<true>(const std::string &,
                                                          const std::map<std::string, std::string> &);
template std::unique_ptr<DSOPP<energy::motion::SE3<Precision>>>
DSOPP<energy::motion::SE3<Precision>>::loadFromYaml<false>(const std::string &,
                                                           const std::map<std::string, std::string> &);
// contact Roadly INC for this functionality
// template class DSOPP<energy::motion::UniformRollingShutter<Precision>>;
// template std::unique_ptr<DSOPP<energy::motion::UniformRollingShutter<Precision>>>
// DSOPP<energy::motion::UniformRollingShutter<Precision>>::loadFromYaml<true>(const std::string &,
//                                                                             const std::map<std::string, std::string>
//                                                                             &);
// template std::unique_ptr<DSOPP<energy::motion::UniformRollingShutter<Precision>>>
// DSOPP<energy::motion::UniformRollingShutter<Precision>>::loadFromYaml<false>(
//     const std::string &, const std::map<std::string, std::string> &);

}  // namespace dsopp
