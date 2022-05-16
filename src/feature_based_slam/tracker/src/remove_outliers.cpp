#include "feature_based_slam/tracker/remove_outliers.hpp"

#include <set>

#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/camera_model/pinhole/simple_radial.hpp"
#include "feature_based_slam/features/distinct_feature.hpp"
#include "feature_based_slam/track/frame.hpp"
#include "feature_based_slam/track/landmarks.hpp"

namespace dsopp {
namespace feature_based_slam {
namespace tracker {

template <energy::model::Model Model>
void removeOutliers(track::Landmarks &landmarks, const Model &model, std::deque<track::Frame> &frames,
                    const size_t last_frames_to_filter, const Precision outlier_threshold) {
  size_t number_of_frames = std::min(frames.size(), last_frames_to_filter);
  size_t start = frames.size() - number_of_frames;
  size_t end = start + number_of_frames;

  std::set<std::pair<size_t, size_t>> to_delete;

  for (size_t i = start; i < end; i++) {
    auto &frame = frames[i];
    if (!frame.initialized) continue;

    for (const auto &landmark_idx : landmarks.getLandmarksFromFrame(frame.frame_id)) {
      const auto &landmark = landmarks.landmarks()[landmark_idx];
      if (landmark.position()) {
        const auto &feature = landmark.projection(frame.frame_id)->coordinates();

        Eigen::Vector2<Precision> projection;
        auto t_agent_world = frame.t_world_agent.inverse();
        bool projected = model.project(t_agent_world * *landmark.position(), projection);

        if (!projected || (feature - projection).norm() > outlier_threshold) {
          to_delete.insert({landmark_idx, frame.frame_id});
        }
      }
    }
  }

  for (const auto &del : to_delete) {
    landmarks.removeProjectionFromLandmark(del.first, del.second);
  }
}

template void removeOutliers<energy::model::PinholeCamera<Precision>>(
    track::Landmarks &landmarks, const energy::model::PinholeCamera<Precision> &model,
    std::deque<track::Frame> &initialization_frames, const size_t, const Precision);

template void removeOutliers<energy::model::SimpleRadialCamera<Precision>>(
    track::Landmarks &, const energy::model::SimpleRadialCamera<Precision> &, std::deque<track::Frame> &, const size_t,
    const Precision);

}  // namespace tracker
}  // namespace feature_based_slam
}  // namespace dsopp
