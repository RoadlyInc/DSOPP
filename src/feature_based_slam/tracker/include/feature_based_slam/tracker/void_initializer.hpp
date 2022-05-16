#ifndef DSOPP_FEATURE_BASED_SLAM_MONOCULAR_VOID_INITIALIZER_HPP_
#define DSOPP_FEATURE_BASED_SLAM_MONOCULAR_VOID_INITIALIZER_HPP_

#include "energy/motion/se3_motion.hpp"
#include "feature_based_slam/features/distinct_features_extractor.hpp"
#include "feature_based_slam/features/distinct_features_frame.hpp"
#include "feature_based_slam/tracker/initializer.hpp"
#include "features/camera/camera_features.hpp"
#include "features/camera/pixel_data_frame.hpp"
#include "features/camera/tracking_features_frame.hpp"

namespace dsopp::feature_based_slam::tracker {

/**
 * \brief Void initialization, which fills ``max_frames_n``
 */
class VoidInitializer : public Initializer {
 public:
  /**
   * @param max_frames_n number of frames to wait to be initialized
   * @param track track which contain all frames and landmarks
   * @param sensor_id sensor id for initialization
   * @param options All constants which will be used in the feature based slam algorithms and optimizations
   * @param feature_extractor feature extractor
   */
  VoidInitializer(const size_t max_frames_n, track::Track &track, const size_t sensor_id,
                  const features::DistinctFeaturesExtractor &feature_extractor, const Options &options)
      : Initializer(track, sensor_id, feature_extractor, options), max_frames_n_(max_frames_n) {}

  void tick(const sensors::SynchronizedFrame &frame_data, const size_t) {
    auto &camera_features = frame_data.cameraFeatures();
    if (!camera_features.contains(sensor_id_)) {
      return;
    }

    auto &frame = *camera_features.at(sensor_id_);
    track_.frames.emplace_back(frame.timestamp(), frame.id(), frame.frameData());
    if (track_.frames.size() >= max_frames_n_) {
      is_initialized_ = true;
    }
  }

  Eigen::VectorX<Precision> cameraIntrinsics() const { return {}; }

 private:
  /** number of frames to wait for be initialized */
  const size_t max_frames_n_;
};

}  // namespace dsopp::feature_based_slam::tracker

#endif  // DSOPP_FEATURE_BASED_SLAM_MONOCULAR_VOID_INITIALIZER_HPP_
