
#ifndef DSOPP_TRACKER_HPP
#define DSOPP_TRACKER_HPP

#include <memory>

#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"
#include "output_interfaces/camera_output_interfaces.hpp"

namespace dsopp {
namespace feature_based_slam {
namespace tracker {
class Tracker;
}
}  // namespace feature_based_slam
namespace track {
template <energy::motion::Motion Motion>
class ActiveTrack;
}  // namespace track
namespace sensors {
class SynchronizedFrame;
}  // namespace sensors
namespace tracker {
/**
 * Tracker for coarse registration of incoming frames
 * @tparam Motion Motion type
 */
template <energy::motion::Motion Motion>
class Tracker {
 public:
  /**
   * method to be called after each synchronization step.
   * @param[in,out] track current track to be updated
   * @param frame_data sensors data from the last tick
   * @param number_of_threads number of threads
   * @param initialization if `true` then tick is called to initialize tracker
   * @param force_keyframe if `true`, keyframe will be created without keyframe strategy
   */
  virtual void tick(track::ActiveTrack<Motion>& track, const sensors::SynchronizedFrame& frame_data,
                    const size_t number_of_threads, bool initialization = false, bool force_keyframe = false) = 0;

  /**
   * sets current frame, keyframe and frame embedding interfaces for the debug images
   * @param camera_output_interface all ouput interfaces
   */
  void setDebugCameraOutputInterface(const output::CameraOutputInterfaces& camera_output_interface) {
    camera_output_interface_ = camera_output_interface;
  }

  /**
   * @return true when scene have been initialized
   */
  bool isInitialized() const { return is_initialized_; }

  /**
   * initialize from the other tracker and frames data
   * @param tracker other tracker that have already initialized
   * @param frames frames data
   * @param track current track to be updated
   * @param number_of_threads number of threads
   */
  virtual void initialize(const feature_based_slam::tracker::Tracker& tracker,
                          const std::vector<std::unique_ptr<sensors::SynchronizedFrame>>& frames,
                          track::ActiveTrack<Motion>& track, const size_t number_of_threads) = 0;

  /**
   * @return true when frame embedder is used
   */
  virtual bool frameEmbedder() const = 0;

  virtual ~Tracker() = default;

 protected:
  /** current frame interface */
  output::CameraOutputInterfaces camera_output_interface_;
  /** true when scene have been initialized */
  bool is_initialized_ = false;
};
}  // namespace tracker
}  // namespace dsopp
#endif  // DSOPP_TRACKER_HPP
