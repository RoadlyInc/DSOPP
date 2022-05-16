#include "feature_based_slam/tracker/initialize_poses.hpp"

#include <glog/logging.h>

#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/camera_model/pinhole/simple_radial.hpp"
#include "feature_based_slam/track/frame.hpp"
#include "feature_based_slam/track/track.hpp"
#include "feature_based_slam/tracker/estimate_se3_pnp.hpp"
#include "feature_based_slam/tracker/estimate_so3_inlier_count.hpp"
#include "feature_based_slam/tracker/estimate_so3xs2.hpp"
#include "feature_based_slam/tracker/refine_track.hpp"
#include "feature_based_slam/tracker/triangulate_points.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"

namespace dsopp {
namespace feature_based_slam {
namespace tracker {

template <energy::model::Model Model>
bool initializePoses(track::Track &track, const Model &camera_model, const Options &options,
                     const size_t number_of_threads) {
  auto &frames = track.frames;
  auto &landmarks = track.landmarks;

  auto &first_frame = frames.front();
  auto &last_frame = frames.back();

  /* I. Estimating SE3 from last to first frame */
  {
    auto [inlier_number, pair_number] = estimateSO3xS2(
        first_frame.frame_id, last_frame.frame_id, landmarks, last_frame.t_world_agent, number_of_threads, camera_model,
        options.min_ransac_matches, options.essential_matrix_ransac_threshold, options.reprojection_threshold);

    VLOG(1) << "SO3xS2 Inliers : " << inlier_number << " (" << pair_number << ")";

    if (inlier_number < static_cast<size_t>(options.se3_inlier_ratio * static_cast<Precision>(pair_number)))
      return false;
    frames.front().initialized = true;
    frames.back().initialized = true;
  }

  /* II. Triangulating points between first and last frames */
  triangulatePoints(first_frame.frame_id, last_frame.frame_id, landmarks, first_frame.t_world_agent,
                    last_frame.t_world_agent, camera_model, options.triangulation_cos_threshold);

  /* III. PnP poses for the remaining frames */
  size_t pnp_initialized_n = 0;
  for (size_t i = 1; i < frames.size() - 1; ++i) {
    auto &frame = frames[i];
    auto [inlier_number, pair_number] =
        estimateSE3PnP(frame.frame_id, landmarks, frame.t_world_agent, camera_model, options.pnp_ransac_threshold);

    VLOG(1) << "PnP Inliers : " << inlier_number << " (" << pair_number << ")";
    if (inlier_number > static_cast<size_t>(options.pnp_inlier_ratio * static_cast<Precision>(pair_number))) {
      frames[i].initialized = true;
      pnp_initialized_n++;
    }
  }
  if (pnp_initialized_n < 1) return false;

  /* IV. Perform non-linear optimization */
  optimizeTransformations<Model>(landmarks, number_of_threads, camera_model.image_size(),
                                 camera_model.intrinsicsParameters(), frames, options.reprojection_threshold);
  /* V. rePnP uninitialized frames */
  for (size_t i = 1; i < frames.size() - 1; ++i) {
    auto &frame = frames[i];
    if (frame.initialized) continue;
    auto [inlier_number, pair_number] =
        estimateSE3PnP(frame.frame_id, landmarks, frame.t_world_agent, camera_model, options.pnp_ransac_threshold);

    VLOG(1) << "Second try PnP Inliers : " << inlier_number << " (" << pair_number << ")";
  }

  return true;
}

template bool initializePoses<energy::model::PinholeCamera<Precision>>(track::Track &,
                                                                       const energy::model::PinholeCamera<Precision> &,
                                                                       const Options &, const size_t);

template bool initializePoses<energy::model::SimpleRadialCamera<Precision>>(
    track::Track &, const energy::model::SimpleRadialCamera<Precision> &, const Options &, const size_t);
}  // namespace tracker
}  // namespace feature_based_slam
}  // namespace dsopp
