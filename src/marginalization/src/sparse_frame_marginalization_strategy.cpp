#include "marginalization/sparse_frame_marginalization_strategy.hpp"

#include <numeric>
#include <set>

#include "track/active_odometry_track.hpp"
#include "track/connections/frame_connection.hpp"
#include "track/frames/active_keyframe.hpp"
#include "track/landmarks/active_tracking_landmark.hpp"
#include "track/landmarks/immature_tracking_landmark.hpp"

namespace dsopp {
namespace marginalization {
namespace {
template <energy::motion::Motion Motion>
size_t numberOfActiveAndImmatureLandmarks(const track::ActiveKeyframe<Motion> &frame) {
  long n = 0;
  for (auto sensor : frame.sensors()) {
    const auto &active_landmarks = frame.activeLandmarks(sensor);
    n += std::ranges::count_if(active_landmarks, [](const auto &l) { return !l.isOutlier() && !l.isMarginalized(); });
    const auto &immature_landmarks = frame.immatureLandmarks(sensor);
    n += std::ranges::count_if(immature_landmarks, [](const auto &l) { return l.readyForActivation(); });
  }

  return static_cast<size_t>(n);
}
template <energy::motion::Motion Motion>
size_t numberOfMarginalizedLandmarks(const track::ActiveKeyframe<Motion> &frame) {
  long n = 0;
  for (auto sensor : frame.sensors()) {
    const auto &active_landmarks = frame.activeLandmarks(sensor);
    n += std::ranges::count_if(active_landmarks, [](const auto &l) { return !l.isOutlier() && l.isMarginalized(); });
  }
  return static_cast<size_t>(n);
}

template <energy::motion::Motion Motion>
void findFramesWithSmallNumberOfActivePoints(const track::ActiveOdometryTrack<Motion> &track,
                                             const size_t keep_frames_from_start,
                                             const double maximum_number_of_marginalized, const size_t minimum_size,
                                             std::set<size_t> &to_marginalize) {
  for (size_t i = 0; i + keep_frames_from_start < track.activeFrames().size(); i++) {
    const auto &frame = track.getActiveKeyframe(i);
    // TODO: also can be tracked the change in exposure
    double number_of_active_and_immature_points = static_cast<double>(numberOfActiveAndImmatureLandmarks(frame));
    double number_of_all_valid_points =
        number_of_active_and_immature_points + static_cast<double>(numberOfMarginalizedLandmarks(frame));
    if (number_of_active_and_immature_points < (1 - maximum_number_of_marginalized) * number_of_all_valid_points &&
        (track.activeFrames().size() - to_marginalize.size() > minimum_size)) {
      to_marginalize.insert(i);
    }
  }
}

template <energy::motion::Motion Motion>
void marginalizeLandmarks(track::ActiveOdometryTrack<Motion> &track,
                          const size_t min_good_residuals_in_the_last_optimization,
                          const size_t number_of_good_optimizations, const size_t keep_frames_from_start) {
  if (track.activeFrames().size() > keep_frames_from_start) {
    size_t last_keyframe_idx = track.activeFrames().back()->keyframeId();
    for (size_t frame_idx = 0; frame_idx < track.activeFrames().size() - 1; frame_idx++) {
      auto &frame = track.getActiveKeyframe(frame_idx);
      for (auto sensor : frame.sensors()) {
        for (size_t landmark_idx = 0; landmark_idx < frame.activeLandmarks(sensor).size(); landmark_idx++) {
          auto &landmark = frame.getActiveLandmark(sensor, landmark_idx);
          if (landmark.isMarginalized() || landmark.isOutlier()) {
            continue;
          }
          bool out_of_boundary = true;
          for (auto target_sensor : frame.sensors()) {
            auto last_status = frame.getConnection(last_keyframe_idx)
                                   .referenceReprojectionStatuses(sensor, target_sensor)
                                   .at(landmark_idx);
            out_of_boundary &= last_status != track::PointConnectionStatus::kOk;
          }
          bool valid_for_marginalization =
              (landmark.numberOfInlierResidualsInTheLastOptimization() >= min_good_residuals_in_the_last_optimization &&
               landmark.numberOfSuccessfulOptimizations() > number_of_good_optimizations);
          if (out_of_boundary || valid_for_marginalization) {
            landmark.marginalize();
          } else if (out_of_boundary && !valid_for_marginalization) {
            landmark.markOutlier();
          }
        }
      }
    }
  }
}
/**
 * If the number of active frames is greater than the maximum, the frames are marginalized using a scoring function.
 * This scoring function is heuristically designed to keep active keyframes well-distributed in 3D space, with more
 * keyframes close to the most recent one
 * For details, see paper 'Direct Sparse Odometry' by Engel et al.
 * https://arxiv.org/pdf/1607.02565.pdf
 * Equation (20)
 */
template <energy::motion::Motion Motion>
void findFramesToMarginalize(const track::ActiveOdometryTrack<Motion> &track, const size_t keep_frames_from_start,
                             const size_t min_frame_age, const size_t maximum_size, std::set<size_t> &to_marginalize) {
  const double kEps = 1e-5;

  if (track.activeFrames().size() > maximum_size + to_marginalize.size()) {
    double max_score = 0;
    size_t to_marginalize_idx = 0;
    const auto &last_keyframe = track.lastKeyframe();

    for (size_t reference_idx = 0; reference_idx + keep_frames_from_start < track.activeFrames().size();
         reference_idx++) {
      const auto &reference_frame = track.getActiveKeyframe(reference_idx);
      if (reference_frame.id() + min_frame_age > last_keyframe.id()) {
        continue;
      }

      double distance_score = 0;
      for (size_t target_idx = 0; target_idx + keep_frames_from_start < track.activeFrames().size(); target_idx++) {
        const auto &target_frame = track.getActiveKeyframe(target_idx);
        if (target_frame.id() + min_frame_age > last_keyframe.id() + 1 || reference_idx == target_idx) {
          continue;
        }
        double distance =
            (reference_frame.tWorldAgent().translation() - target_frame.tWorldAgent().translation()).norm();
        distance_score += 1 / (kEps + distance);
      }
      double distance =
          (reference_frame.tWorldAgent().translation() - track.activeFrames().back()->tWorldAgent().translation())
              .norm();
      distance_score *= sqrt(distance);

      if (distance_score > max_score) {
        max_score = distance_score;
        to_marginalize_idx = reference_idx;
      }
    }

    to_marginalize.insert(to_marginalize_idx);
  }
}
}  // namespace

template <energy::motion::Motion Motion>
SparseFrameMarginalizationStrategy<Motion>::SparseFrameMarginalizationStrategy(size_t minimum_size, size_t maximum_size,
                                                                               double maximum_number_of_marginalized)
    : minimum_size_(minimum_size),
      maximum_size_(maximum_size),
      maximum_number_of_marginalized_(maximum_number_of_marginalized) {}

template <energy::motion::Motion Motion>
void SparseFrameMarginalizationStrategy<Motion>::marginalize(track::ActiveOdometryTrack<Motion> &track) {
  const size_t kMinFrameAge = 1;
  const size_t kKeepFramesFromStart = 2;
  const size_t kMinGoodResidualsInTheLastOptimization = (minimum_size_ + 1) / 2;
  const size_t kNumberOfGoodOptimizations = maximum_size_ * 2;
  std::set<size_t> to_marginalize;

  findFramesWithSmallNumberOfActivePoints(track, kKeepFramesFromStart, maximum_number_of_marginalized_, minimum_size_,
                                          to_marginalize);
  findFramesToMarginalize(track, kKeepFramesFromStart, kMinFrameAge, maximum_size_, to_marginalize);
  marginalizeLandmarks(track, kMinGoodResidualsInTheLastOptimization, kNumberOfGoodOptimizations, kKeepFramesFromStart);

  // marginalize
  for (const auto &to_marginalize_idx : to_marginalize) {
    track.marginalizeFrame(to_marginalize_idx);
  }
}

template <energy::motion::Motion Motion>
SparseFrameMarginalizationStrategy<Motion>::~SparseFrameMarginalizationStrategy() = default;

template class SparseFrameMarginalizationStrategy<energy::motion::SE3<Precision>>;
}  // namespace marginalization
}  // namespace dsopp
