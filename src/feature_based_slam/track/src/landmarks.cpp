#include "feature_based_slam/track/landmarks.hpp"

#include <glog/logging.h>

namespace dsopp {
namespace feature_based_slam {
namespace track {

size_t Landmarks::add() {
  landmarks_.emplace_back();
  return landmarks_.size() - 1;
}

size_t Landmarks::add(const Eigen::Vector3<Precision> &position) {
  landmarks_.emplace_back(position);
  return landmarks_.size() - 1;
}

const std::deque<Landmark> &Landmarks::landmarks() const { return landmarks_; }

void Landmarks::setPositionOfLandmark(const size_t idx, const Eigen::Vector3<Precision> &position) {
  CHECK_LT(idx, landmarks_.size());
  landmarks_[idx].setPosition(position);
}

void Landmarks::addProjectionToLandmark(const size_t idx, size_t frame_id, const features::DistinctFeature &feature) {
  CHECK_LT(idx, landmarks_.size());
  landmarks_[idx].addProjection(frame_id, feature);
  landmarks_on_frame_[frame_id].push_back(idx);
}

void Landmarks::removeProjectionFromLandmark(const size_t idx, size_t frame_id) {
  CHECK_LT(idx, landmarks_.size());
  landmarks_[idx].removeProjection(frame_id);
  landmarks_on_frame_[frame_id].erase(
      std::remove(landmarks_on_frame_[frame_id].begin(), landmarks_on_frame_[frame_id].end(), idx),
      landmarks_on_frame_[frame_id].end());
}

std::vector<size_t> Landmarks::getLandmarksFromTwoFrames(size_t frame_id_1, size_t frame_id_2) {
  std::vector<size_t> result;
  for (const auto &idx : landmarks_on_frame_[frame_id_1]) {
    if (landmarks_[idx].projection(frame_id_2)) {
      result.push_back(idx);
    }
  }
  return result;
}

const std::vector<size_t> &Landmarks::getLandmarksFromFrame(size_t frame_id) { return landmarks_on_frame_[frame_id]; }

}  // namespace track
}  // namespace feature_based_slam
}  // namespace dsopp
