#include "track/active_odometry_track.hpp"

#include <glog/logging.h>
#include <tbb/parallel_for.h>

#include "track/connections/connections_container.hpp"
#include "track/connections/frame_connection.hpp"
#include "track/frames/active_keyframe.hpp"
#include "track/frames/keyframe.hpp"
#include "track/landmarks/active_tracking_landmark.hpp"
#include "track/landmarks/immature_tracking_landmark.hpp"
#include "track/landmarks/tracking_landmark.hpp"
#include "track/odometry_track.hpp"

namespace dsopp {
namespace track {

template <energy::motion::Motion Motion>
ActiveOdometryTrack<Motion>::ActiveOdometryTrack() = default;

template <energy::motion::Motion Motion>
void ActiveOdometryTrack<Motion>::pushFrame(size_t id, time timestamp, const Motion &t_world_agent,
                                            const Precision exposure_time,
                                            const Eigen::Vector2<Precision> &affine_brightness) {
  frames_.push_back(std::make_unique<track::ActiveKeyframe<Motion>>(id, frames_.size(), timestamp, t_world_agent,
                                                                    exposure_time, affine_brightness));
  active_frames_.emplace_back(frames_.back().get());
}

template <energy::motion::Motion Motion>
const std::vector<ActiveKeyframe<Motion> *> ActiveOdometryTrack<Motion>::keyframes() const {
  std::vector<ActiveKeyframe<Motion> *> frames;
  for (const auto &frame : frames_) {
    frames.push_back(frame.get());
  }
  return frames;
}

template <energy::motion::Motion Motion>
ActiveKeyframe<Motion> &ActiveOdometryTrack<Motion>::lastKeyframe() const {
  CHECK(!frames_.empty());
  return *frames_.back();
}

template <energy::motion::Motion Motion>
const std::deque<ActiveKeyframe<Motion> *> &ActiveOdometryTrack<Motion>::marginalizedFrames() const {
  return marginalized_frames_;
}

template <energy::motion::Motion Motion>
const ActiveKeyframe<Motion> &ActiveOdometryTrack<Motion>::marginalizedFrame(size_t index) const {
  return *marginalized_frames_[index];
}

template <energy::motion::Motion Motion>
bool ActiveOdometryTrack<Motion>::empty() const {
  return keyframes().empty();
}

template <energy::motion::Motion Motion>
const std::vector<const ActiveKeyframe<Motion> *> ActiveOdometryTrack<Motion>::frames() const {
  std::vector<const ActiveKeyframe<Motion> *> frames;
  for (const auto &active_frame : frames_) {
    frames.push_back(active_frame.get());
  }
  return frames;
}

template <energy::motion::Motion Motion>
void ActiveOdometryTrack<Motion>::marginalizeFrame(size_t active_frame_id) {
  auto frame_ptr = active_frames_[active_frame_id];
  frame_ptr->marginalize();
  active_frames_.erase(active_frames_.begin() + static_cast<int>(active_frame_id));
  marginalized_frames_.push_back(frame_ptr);
}

template <energy::motion::Motion Motion>
void ActiveOdometryTrack<Motion>::unloadMarginalizedResources() {
  for (auto it = marginalized_frames_.rbegin(); it != marginalized_frames_.rend() && (*it)->isMarginalized(); ++it) {
    (*it)->unloadResources();
  }
}

template <energy::motion::Motion Motion>
ActiveKeyframe<Motion> &ActiveOdometryTrack<Motion>::getActiveKeyframe(const size_t frame_id) const {
  return *active_frames_.at(frame_id);
}

template <energy::motion::Motion Motion>
void ActiveOdometryTrack<Motion>::applyImmatureLandmarkActivationStatuses(
    std::map<size_t, ImmatureLandmarkActivationStatusesPerSensor> &statuses) {
  std::map<size_t, std::map<size_t, size_t>> activated_landmarks_per_frame;
  for (auto &frame : active_frames_) {
    if (statuses.contains(frame->id())) {
      activated_landmarks_per_frame[frame->id()] =
          frame->applyImmatureLandmarkActivationStatuses(statuses.at(frame->id()));
    }
  }

  for (auto &reference_frame : active_frames_) {
    for (auto &target_frame : active_frames_) {
      if (reference_frame->id() >= target_frame->id()) {
        continue;
      }
      for (const auto &sensor : reference_frame->sensors()) {
        if (this->connections_->exists(reference_frame->keyframeId(), target_frame->keyframeId())) {
          auto &connection = this->connections_->get(reference_frame->keyframeId(), target_frame->keyframeId());
          connection.addSensorConnection(sensor, sensor,
                                         activated_landmarks_per_frame.at(reference_frame->id()).at(sensor),
                                         activated_landmarks_per_frame.at(target_frame->id()).at(sensor));
        } else {
          auto connection = std::make_unique<FrameConnection<typename Motion::Product>>(reference_frame->keyframeId(),
                                                                                        target_frame->keyframeId());
          connection->addSensorConnection(sensor, sensor, 0, target_frame->activeLandmarks(sensor).size());
          const auto &reference_landmarks = reference_frame->activeLandmarks(sensor);
          std::vector<PointConnectionStatus> reference_connections(reference_landmarks.size(),
                                                                   PointConnectionStatus::kOk);
          for (size_t i = 0; i < reference_landmarks.size(); i++) {
            if (reference_landmarks[i].isMarginalized() || reference_landmarks[i].isOutlier()) {
              reference_connections[i] = PointConnectionStatus::kUnknown;
            }
          }
          connection->setReferenceReprojectionStatuses(sensor, sensor, reference_connections);
          reference_frame->addConnection(target_frame->keyframeId(), connection.get());
          target_frame->addConnection(reference_frame->keyframeId(), connection.get());
          this->connections_->add(std::move(connection));
        }
      }
    }
  }
}

namespace {

template <energy::motion::Motion Motion>
const Frame<Motion> *tryToFindInKeyframe(const ActiveKeyframe<Motion> *keyframe, size_t &step) {
  if (keyframe) {
    if (keyframe->attachedFrames().size() > step) {
      return keyframe->attachedFrames()[keyframe->attachedFrames().size() - 1 - step].get();
    } else if (keyframe->attachedFrames().size() == step) {
      return keyframe;
    } else {
      step -= keyframe->attachedFrames().size() + 1;
    }
  }
  return nullptr;
}
}  // namespace

template <energy::motion::Motion Motion>
const Frame<Motion> *ActiveOdometryTrack<Motion>::getFrame(int n) const {
  const Frame<Motion> *frame = nullptr;
  if (empty()) {
    LOG(ERROR) << "Can't get " << n << "th frame in the track. Track is empty";
    return nullptr;
  }
  if (n >= 0) {
    LOG(ERROR) << "Get frame for n >= 0 is not implemented";
    return nullptr;
  }
  size_t step = static_cast<size_t>(std::abs(n + 1));
  for (int i = static_cast<int>(frames_.size()) - 1; i >= 0; i--) {
    if ((frame = tryToFindInKeyframe(frames_[static_cast<size_t>(i)].get(), step))) {
      return frame;
    }
  }
  LOG(ERROR) << "There is no " << n << "th frame in the track";
  return nullptr;
}

template <energy::motion::Motion Motion>
const std::deque<ActiveKeyframe<Motion> *> &ActiveOdometryTrack<Motion>::activeFrames() const {
  return active_frames_;
}

template <energy::motion::Motion Motion>
std::unique_ptr<OdometryTrack<Motion>> ActiveOdometryTrack<Motion>::createTrack(
    const std::map<size_t, const semantics::SemanticLegend *> &legends) const {
  std::vector<std::unique_ptr<Keyframe<Motion>>> keyframes(frames_.size());

  tbb::parallel_for<tbb::blocked_range<size_t>>(tbb::blocked_range<size_t>(0, frames_.size()), [&](auto &r) {
    for (size_t frame_i = r.begin(); frame_i != r.end(); ++frame_i) {
      auto &frame = frames_[frame_i];
      auto sensors = frame->sensors();

      typename Keyframe<Motion>::LandmarksFrame landmarks;
      for (const auto &sensor : sensors) {
        landmarks.insert({sensor, {}});
        landmarks[sensor].reserve(frame->activeLandmarks(sensor).size());

        bool legend_found = legends.contains(sensor);
        for (const auto &landmark : frame->activeLandmarks(sensor)) {
          landmarks.at(sensor).emplace_back(landmark.direction(), landmark.projection(), landmark.idepth(),
                                            landmark.idepthVariance(), landmark.relativeBaseline(),
                                            legend_found ? landmark.semanticTypeId(legends.at(sensor)) : 0);
        }
      }

      keyframes[frame_i] =
          std::make_unique<Keyframe<Motion>>(frame->id(), frame->keyframeId(), frame->timestamp(), frame->tWorldAgent(),
                                             frame->exposureTime(), frame->affineBrightness(), std::move(landmarks));
      for (const auto &sensor : sensors) {
        keyframes[frame_i]->pushImage(sensor, frame->image(sensor));
      }

      for (const auto &attached_frame : frame->attachedFrames()) {
        keyframes[frame_i]->attachTrackingFrame(attached_frame->id(), attached_frame->timestamp(),
                                                attached_frame->tKeyframeAgent(), attached_frame->exposureTime(),
                                                attached_frame->affineBrightness());
        for (const auto &sensor : sensors) {
          keyframes[frame_i]->lastAttachedFrame().pushImage(sensor, attached_frame->image(sensor));
        }
      }
    }
  });

  return std::make_unique<OdometryTrack<Motion>>(
      std::move(keyframes), std::make_unique<ConnectionsContainer<typename Motion::Product>>(*this->connections_));
}

template <energy::motion::Motion Motion>
ActiveOdometryTrack<Motion>::~ActiveOdometryTrack() = default;

template class ActiveOdometryTrack<energy::motion::SE3<Precision>>;

}  // namespace track
}  // namespace dsopp
