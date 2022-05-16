#ifndef DSOPP_TRACKER_CREATE_DEPTH_MAPS_HPP_
#define DSOPP_TRACKER_CREATE_DEPTH_MAPS_HPP_

#include <vector>

#include "energy/motion/motion.hpp"
#include "energy/problems/depth_map.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"
#include "track/frames/active_keyframe.hpp"

namespace dsopp::tracker {

/**
 * initialize empty depth maps
 *
 * @param pyramid pyramids
 * @param[out] idepth_maps vector to be filled with empty maps
 */
void initDepthMaps(const features::Pyramid &pyramid, std::vector<energy::problem::DepthMap> &idepth_maps);

/**
 * fill coarse levels of depth maps (1 to ``depth_maps.size()``)
 * by bi-linear interpolation of depth map of finest level
 *
 * @param[out] depth_maps depth maps
 */
void fillCoarseDepthMaps(std::vector<energy::problem::DepthMap> &depth_maps);

/**
 * @param[out] depth_maps depth maps
 */
void dilateDepthMaps(std::vector<energy::problem::DepthMap> &depth_maps);

/**
 * create depth maps for last frame in ``frames``
 *
 * @tparam Motion motion type
 * @tparam Calibration camera calibration type
 * @param frames frames
 * @param calibration calibration
 */
template <energy::motion::Motion Motion, energy::model::Model Model>
std::map<size_t, std::vector<energy::problem::DepthMap>> createReferenceDepthMaps(
    const std::deque<track::ActiveKeyframe<Motion> *> &frames,
    const sensors::calibration::CameraCalibration &calibration);

}  // namespace dsopp::tracker

#endif  // DSOPP_TRACKER_CREATE_DEPTH_MAPS_HPP_
