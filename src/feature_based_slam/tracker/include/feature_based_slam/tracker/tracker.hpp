#ifndef DSOPP_FEATURE_BASED_SLAM_TRACKER_TRACKER_HPP
#define DSOPP_FEATURE_BASED_SLAM_TRACKER_TRACKER_HPP

#include <functional>

#include "feature_based_slam/features/distinct_features_extractor.hpp"
#include "feature_based_slam/initialization_strategy/initializer_keyframe_strategy.hpp"
#include "feature_based_slam/track/track.hpp"
#include "feature_based_slam/tracker/initializer.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"

namespace dsopp {
namespace feature_based_slam {
namespace tracker {
class Initializer;
/**
 * Feature based slam tracker
 */
class Tracker {
 public:
  /**
   * Creates features based slam tracker.
   * @param sensor_id sensor id for initialization
   * @param feature_extractor feature extractor and matcherer
   * @param initializer_fabric initializer fabric
   * @param options all constants which will be used in the feature based slam algorithms and optimizations
   * */
  Tracker(const size_t sensor_id, std::unique_ptr<features::DistinctFeaturesExtractor> &&feature_extractor,
          std::function<std::unique_ptr<Initializer>(track::Track &, const features::DistinctFeaturesExtractor &)>
              &initializer_fabric,
          const Options &options = Options());

  /**
   * method to clear internal data of tracker
   */
  void clear();

  /**
   * method to push new frame to tracker
   *
   * @param frame_data feature frame contatins all frame data
   * @param number_of_threads number of threads
   * @return bool flag, true if more frames are required to initializer
   */
  virtual void tick(const sensors::SynchronizedFrame &frame_data, const size_t number_of_threads) = 0;

  /**
   * @return true if initialization was successful
   */
  bool initialized();

  /**
   * method to get number of estimated frames
   * @return number of estimated frames
   */
  size_t evaluatedFramesSize() const;

  /**
   * method to get agent to world pose from idx
   * @param idx frame index
   * @return agent pose in the world
   */
  const energy::motion::SE3<Precision> &tWorldAgent(size_t idx) const;

  /**
   * method to get frame timestamp by idx
   * @param idx frame index
   * @return timestamp
   */
  time timestamp(size_t idx) const;

  /**
   * method to get indicator of frames sinitialization success
   * @param idx frame index
   * @return initialization indicator
   */
  bool isInitialized(size_t idx) const;
  /**
   * @return intrinsics parameters of model that was used
   */
  virtual Eigen::VectorX<Precision> cameraIntrinsics() const = 0;
  virtual ~Tracker();

 protected:
  /** sensor id for initialization */
  const size_t sensor_id_;
  /** feature extractor */
  std::unique_ptr<features::DistinctFeaturesExtractor> feature_extractor_;
  /** initializer find first poses of frames and first landmarks */
  std::unique_ptr<Initializer> initializer_;
  /** track which contain all frames and landmarks */
  track::Track track_;
  /** All constants which will be used in the feature based slam algorithms and optimizations */
  const Options options_;
};

}  // namespace tracker
}  // namespace feature_based_slam
}  // namespace dsopp

#endif  // DSOPP_FEATURE_BASED_SLAM_TRACKER_TRACKER_HPP
