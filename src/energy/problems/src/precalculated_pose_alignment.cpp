#include "energy/problems/pose_alignment/precalculated_pose_alignment.hpp"

#include <glog/logging.h>
#include <chrono>
#include <limits>

#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/camera_model/pinhole/simple_radial.hpp"
#include "energy/motion/se3_motion.hpp"

#include "energy/problems/photometric_bundle_adjustment/local_frame.hpp"
#include "features/camera/ceres_grid.hpp"
#include "features/camera/pixel_map.hpp"

namespace dsopp::energy::problem {

template <energy::motion::Motion Motion, model::Model Model, int PatternSize, int C>
PrecalculatedPoseAlignment<Motion, Model, PatternSize, C>::PrecalculatedPoseAlignment(
    std::map<time, Motion> &&timestamp_t_w_agents) {
  this->timestamp_t_w_agents_ = common::timestamp_storage::TimestampStorage<Motion>(std::move(timestamp_t_w_agents));
}

template <energy::motion::Motion Motion, model::Model Model, int PatternSize, int C>
void PrecalculatedPoseAlignment<Motion, Model, PatternSize, C>::reset() {
  this->frames_.clear();
}
template <energy::motion::Motion Motion, model::Model Model, int PatternSize, int C>
void PrecalculatedPoseAlignment<Motion, Model, PatternSize, C>::pushKnownPose(time timestamp, const Motion &) {
  if (!this->timestamp_t_w_agents_.getData(timestamp)) {
    LOG(ERROR) << "Pose at " << timestamp << " is not in timestamp_t_w_agents_";
  }
}

template <energy::motion::Motion Motion, model::Model Model, int PatternSize, int C>
Precision PrecalculatedPoseAlignment<Motion, Model, PatternSize, C>::solve(const size_t) {
  auto &target_frame = *this->frames_.back();

  auto nearest_t_w_t = this->timestamp_t_w_agents_.getData(target_frame.timestamp);

  if (nearest_t_w_t) {
    target_frame.T_w_agent_linearization_point = *nearest_t_w_t;
    target_frame.state_eps.setZero();

    return PoseAlignment<Motion, Model, PatternSize, features::PixelMap, C>::kZeroCost;
  }

  LOG(ERROR) << "Pose for timestamp " << target_frame.timestamp << " was not in timestamp_t_w_agents_ base";
  return std::numeric_limits<Precision>::max();
}

#define PoseAlignmentPrecalcultedInstantiation(Motion, Model, PatchSize) \
  template class PrecalculatedPoseAlignment<energy::motion::Motion<Precision>, model::Model<Precision>, PatchSize>

PoseAlignmentPrecalcultedInstantiation(SE3, PinholeCamera, 8);
PoseAlignmentPrecalcultedInstantiation(SE3, PinholeCamera, 1);
PoseAlignmentPrecalcultedInstantiation(SE3, SimpleRadialCamera, 1);
#undef PoseAlignmentPrecalcultedInstantiation

}  // namespace dsopp::energy::problem
