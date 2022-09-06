#include "track/frames/slam_internal_tracking_frame.hpp"

namespace dsopp {
namespace track {
template <energy::motion::Motion Motion>
SLAMInternalTrackingFrame<Motion>::SLAMInternalTrackingFrame(size_t id, time timestamp, const Motion& t_world_keyframe,
                                                             const typename Motion::Product& t_keyframe_agent,
                                                             const Precision exposure_time,
                                                             const Eigen::Vector<Precision, 2>& affine_brightness,
                                                             Precision mean_square_optical_flow,
                                                             Precision mean_square_optical_flow_without_rotation,
                                                             Precision pose_rmse, bool reliable)
    : TrackingFrame<Motion>(id, timestamp, t_world_keyframe, t_keyframe_agent, exposure_time, affine_brightness),
      mean_square_optical_flow_(mean_square_optical_flow),
      mean_square_optical_flow_without_rotation_(mean_square_optical_flow_without_rotation),
      pose_rmse_(pose_rmse),
      reliable_(reliable) {}

template <energy::motion::Motion Motion>
Precision SLAMInternalTrackingFrame<Motion>::meanSquareOpticalFlow() const {
  return mean_square_optical_flow_;
}

template <energy::motion::Motion Motion>
Precision SLAMInternalTrackingFrame<Motion>::poseRmse() const {
  return pose_rmse_;
}

template <energy::motion::Motion Motion>
bool SLAMInternalTrackingFrame<Motion>::reliable() const {
  return reliable_;
}

template <energy::motion::Motion Motion>
Precision SLAMInternalTrackingFrame<Motion>::meanSquareOpticalFlowWithoutRotation() const {
  return mean_square_optical_flow_without_rotation_;
}

template class SLAMInternalTrackingFrame<energy::motion::SE3<Precision>>;

}  // namespace track
}  // namespace dsopp
