
#ifndef DSOPP_DSOPP_HPP
#define DSOPP_DSOPP_HPP

#include <map>
#include <memory>
#include <vector>

#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"

namespace dsopp {
namespace feature_based_slam {
namespace tracker {
class Tracker;
}
}  // namespace feature_based_slam
class Agent;
namespace synchronizer {
class Synchronizer;
}  // namespace synchronizer
namespace tracker {
template <energy::motion::Motion Motion>
class Tracker;
}  // namespace tracker
namespace global_localization {
class GlobalLocalization;
}  // namespace global_localization
namespace sanity_checker {
template <template <class> class TrackType, energy::motion::Motion Motion>
class SanityChecker;
}  // namespace sanity_checker
namespace track {
template <energy::motion::Motion Motion>
class ActiveOdometryTrack;
template <energy::motion::Motion Motion>
class ActiveTrack;
}  // namespace track
namespace output {
template <template <class> class OdometryTrackType, energy::motion::Motion Motion>
class TrackOutputInterface;
struct CameraOutputInterfaces;
class TextOutputInterface;
}  // namespace output
/**
 * \brief Main application.
 *
 * Runs DSO++ SLAM.
 * @tparam Motion Motion
 */
template <energy::motion::Motion Motion>
class DSOPP {
 public:
  /** Creates main DSO++ application
   * @param track active track that stores agent settings and all type of tracks (odometry, ecf, etc.)
   * @param agent loaded Agent
   * @param synchronizer frame synchronizer to manage system time
   * @param tracker main direct SLAM tracker
   * @param feature_based_slam_tracker feature based slam tracker to initialize direct SLAM tracker
   * @param sanity_checker sanity checker for track sanity checks
   */
  DSOPP(std::unique_ptr<track::ActiveTrack<Motion>> &&track, std::unique_ptr<Agent> &&agent,
        std::unique_ptr<synchronizer::Synchronizer> &&synchronizer, std::unique_ptr<tracker::Tracker<Motion>> &&tracker,
        std::unique_ptr<feature_based_slam::tracker::Tracker> &&feature_based_slam_tracker,
        std::unique_ptr<sanity_checker::SanityChecker<track::ActiveTrack, Motion>> &&sanity_checker);
  /**
   * loads application configuration from the yaml file
   *
   * @param file path to yaml file
   * @param config_args argument to replace values from the config file
   * @return DSOPP object or nullptr on failure
   */
  template <bool TRANSFORM_TO_PINHOLE = true>
  static std::unique_ptr<DSOPP<Motion>> loadFromYaml(const std::string &file,
                                                     const std::map<std::string, std::string> &config_args = {});
  /**
   * Runs application
   * @param number_of_threads number of available threads
   */
  void run(const size_t number_of_threads);
  /**
   * Method to optimize calibration
   * @param number_of_threads
   * @return string with the camera model parameters
   */
  std::string refineCalibration(const size_t number_of_threads);
  /**
   * Push new output interface to the application
   * @param track_output_interface new output interface
   */
  void addTrackOutputInterface(
      output::TrackOutputInterface<track::ActiveOdometryTrack, Motion> &track_output_interface);
  /**
   * Push output interface for debug image
   * @param output_interfaces camera output interfaces
   */
  void addDebugCameraOutputInterfaces(const output::CameraOutputInterfaces &output_interfaces);
  /**
   * Push output interface for text based info
   * @param name name of the interface
   * @param output_interface output interface
   */
  void addTextOutputInterface(const std::string &name, output::TextOutputInterface *output_interface);

  /**
   * @return agent
   */
  const Agent &agent() const;

  ~DSOPP();

  /**
   * method to access track
   *
   * @return const reference to track
   */
  const track::ActiveTrack<Motion> &track() const;

 private:
  /** object contains sensors' locations*/
  std::unique_ptr<Agent> agent_;
  /** Synchronizer that manage system time for sensor synchronization*/
  std::unique_ptr<synchronizer::Synchronizer> synchronizer_;
  /** Main direct SLAM tracker */
  std::unique_ptr<tracker::Tracker<Motion>> tracker_;
  /** Feature based slam tracker to initialize direct SLAM tracker */
  std::unique_ptr<feature_based_slam::tracker::Tracker> feature_based_slam_tracker_;
  /** Sanity checker for track sanity checks */
  std::unique_ptr<sanity_checker::SanityChecker<track::ActiveTrack, Motion>> sanity_checker_;
  /** Current track of the agent */
  std::unique_ptr<track::ActiveTrack<Motion>> track_;
  /** Output interfaces */
  std::vector<std::reference_wrapper<output::TrackOutputInterface<track::ActiveOdometryTrack, Motion>>>
      track_output_interfaces_;
  /** interface for text outputs*/
  std::map<std::string, output::TextOutputInterface *> text_interfaces_;
};

}  // namespace dsopp
#endif  // DSOPP_DSOPP_HPP
