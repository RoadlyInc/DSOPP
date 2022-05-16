#ifndef DSOPP_FEATURE_BASED_SLAM_TRACKER_ADD_NEW_FRAME_HPP
#define DSOPP_FEATURE_BASED_SLAM_TRACKER_ADD_NEW_FRAME_HPP

#include "energy/camera_model/camera_model_base.hpp"

namespace dsopp {
namespace features {
class CameraFeatures;
}
namespace feature_based_slam {
namespace features {
class DistinctFeaturesExtractor;
}
namespace track {
struct Track;
}
namespace tracker {
/**
 * Add new frame to the track. It consists of features matching, connecting with the landmarks and creating
 * keyframe/frame.
 *
 * @param track track with the frames and landmarks
 * @param frame camera features frame
 * @param model camera model
 * @param feature_extractor class to extract features on the new frame
 * @param reprojection_threshold used in the so3 estimating
 * @param rotation_ransac_iterations number of iterations in the so3 estimating
 */
template <energy::model::Model Model>
void addNewFrame(track::Track &track, dsopp::features::CameraFeatures &frame, const Model &model,
                 const features::DistinctFeaturesExtractor &feature_extractor, Precision reprojection_threshold,
                 size_t rotation_ransac_iterations);
}  // namespace tracker
}  // namespace feature_based_slam
}  // namespace dsopp

#endif  // DSOPP_FEATURE_BASED_SLAM_TRACKER_ADD_NEW_FRAME_HPP
