#include "feature_based_slam/tracker/feature_frame_from_landmarks.hpp"

#include "feature_based_slam/features/correspondence.hpp"
#include "feature_based_slam/track/frame.hpp"
#include "feature_based_slam/track/landmarks.hpp"
#include "feature_based_slam/tracker/landmark_feature_frame.hpp"

namespace dsopp {
namespace feature_based_slam {
namespace tracker {

namespace {
const track::Frame *getFrame(std::deque<track::Frame> &frames, size_t frame_id) {
  auto found = std::find_if(frames.begin(), frames.end(), [frame_id](auto &a) { return a.frame_id == frame_id; });
  return found != frames.end() ? &(*found) : nullptr;
}
}  // namespace

std::vector<LandmarkFeatureFrame> featureFrameFromLandmarks(std::deque<track::Frame> &frames,
                                                            track::Landmarks &landmarks, size_t frame_id) {
  std::vector<LandmarkFeatureFrame> feature_frames(1);
  const auto *reference_frame = getFrame(frames, frame_id);

  if (!reference_frame) {
    return feature_frames;
  }
  std::vector<features::DistinctFeature> distinct_features;
  std::vector<size_t> landmark_idx;

  const auto &landmarks_vector = landmarks.landmarks();
  for (const auto &idx : landmarks.getLandmarksFromFrame(reference_frame->frame_id)) {
    distinct_features.emplace_back(landmarks_vector[idx].projection(reference_frame->frame_id)->coordinates());
    landmark_idx.push_back(idx);
  }
  feature_frames[0].image = reference_frame->image;
  feature_frames[0].features =
      std::make_unique<features::DistinctFeaturesFrame>(std::move(distinct_features), cv::Mat());
  feature_frames[0].landmark_idx = landmark_idx;
  return feature_frames;
}

}  // namespace tracker
}  // namespace feature_based_slam
}  // namespace dsopp
