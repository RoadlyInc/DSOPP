#include "energy/problems/geometric_bundle_adjustment/local_frame.hpp"

namespace dsopp::energy::problem::geometric_bundle_adjustment {

LocalFrame::LocalFrame(size_t frame_id, const motion::SE3<double> t_agent_world_)
    : t_agent_world(t_agent_world_), frame_id_(frame_id) {}

void LocalFrame::pushObservation(const Eigen::Vector2d &pixel_observation, Eigen::Vector3d &point_3d) {
  observed_points_.push_back(pixel_observation);
  associated_point_3d_.push_back(point_3d);
}

size_t LocalFrame::size() const { return observed_points_.size(); }

std::pair<Eigen::Vector2d, Eigen::Vector3d &> LocalFrame::point(size_t idx) {
  return {observed_points_[idx], associated_point_3d_[idx]};
}

size_t LocalFrame::frameId() const { return frame_id_; }

}  // namespace dsopp::energy::problem::geometric_bundle_adjustment
