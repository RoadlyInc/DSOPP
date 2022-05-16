#ifndef DSOPP_FEATURE_BASED_SLAM_TRACKER_INITIALIZER_HPP
#define DSOPP_FEATURE_BASED_SLAM_TRACKER_INITIALIZER_HPP

#include <memory>

#include <Eigen/Dense>
#include <sophus/so3.hpp>

#include "common/settings.hpp"
#include "common/time/time.hpp"
#include "energy/motion/se3_motion.hpp"
#include "feature_based_slam/features/correspondences_finder.hpp"
#include "feature_based_slam/initialization_strategy/initializer_keyframe_strategy.hpp"
#include "feature_based_slam/track/track.hpp"
#include "sensor/synchronized_frame.hpp"

namespace dsopp::feature_based_slam::tracker {

/**
 * All constants which will be used in the feature based slam algorithms and optimizations
 *
 */
struct Options {
  /** reprojection threshold for SO3 fit and sampson error in pixels */
  Precision reprojection_threshold = 1.5;
  /** number of iterations in SO3 only fitting */
  size_t rotation_ransac_iterations = 200;
  /** minium number of matches to start Essential matrix RANSAC */
  size_t min_ransac_matches = 50;
  /** inlier ratio not to reject SO3xS3 Essential matrix hypotesis */
  Precision se3_inlier_ratio = 0.6_p;
  /** threshold to be inlier in essentail matrix RANSAC from opengv in pixels */
  Precision essential_matrix_ransac_threshold = 0.5;
  /** threshold for triangulation. cos between point and camera bearing vector */
  Precision triangulation_cos_threshold = 0.9999_p;
  /** inlier ratio not to reject PnP SE3 hypotesis */
  Precision pnp_inlier_ratio = 0.5;
  /** threshold to be inlier in PnP RANSAC from opengv in pixels */
  Precision pnp_ransac_threshold = 0.5;
};

/**
 * \brief class for Initialize monocular reconstruction
 *
 * Initialized via Essential Matrix + triangulation + PnP
 * has build-in rotation ransac solver to exclude frames with small translations from optimization
 *
 */

class Initializer {
 public:
  /**
   * Constructor to fill common members of children
   * @param track track which contain all frames and landmarks
   * @param sensor_id sensor id for initialization
   * @param options All constants which will be used in the feature based slam algorithms and optimizations
   * @param feature_extractor feature extractor
   */
  Initializer(track::Track &track, const size_t sensor_id, const features::DistinctFeaturesExtractor &feature_extractor,
              const Options &options)
      : track_(track), sensor_id_(sensor_id), options_(options), feature_extractor_(feature_extractor) {}
  /**
   * method to clear internal data of initializer
   */
  virtual void clear() {
    is_initialized_ = false;
    // TODO: Do not delete track here
    track_ = track::Track();
  };

  /**
   * method to push new frame to initializer
   *
   * @param frame_data feature frame contatins all frame data
   * @param number_of_threads number of threads
   * @return bool flag, true if more frames are required to initializer
   */
  virtual void tick(const sensors::SynchronizedFrame &frame_data, const size_t number_of_threads) = 0;

  /**
   * @return true if initialization was successful
   */
  virtual bool initialized() { return is_initialized_; };
  /**
   * @return intrinsics parameters of model that was used
   */
  virtual Eigen::VectorX<Precision> cameraIntrinsics() const = 0;

  virtual ~Initializer() = default;

 protected:
  /** true when scene have been initialized */
  bool is_initialized_ = false;
  /** track which contain all frames and landmarks */
  track::Track &track_;
  /** sensor id for initialization */
  const size_t sensor_id_;
  /** All constants which will be used in the feature based slam algorithms and optimizations */
  const Options options_;
  /** feature extractor */
  const features::DistinctFeaturesExtractor &feature_extractor_;
};

}  // namespace dsopp::feature_based_slam::tracker

#endif  // DSOPP_FEATURE_BASED_SLAM_TRACKER_INITIALIZER_HPP
