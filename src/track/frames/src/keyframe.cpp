#include "track/frames/keyframe.hpp"

#include <glog/logging.h>
#include <memory>
#include <opencv2/imgcodecs.hpp>

#include "track/frames/tracking_frame.hpp"
#include "track/landmarks/tracking_landmark.hpp"

namespace dsopp {
namespace track {
template <energy::motion::Motion Motion>
Keyframe<Motion>::Keyframe(size_t id, size_t keyframe_id, time timestamp, const Motion &tWorldAgent,
                           const Eigen::Vector<Precision, 2> &affine_brightness, LandmarksFrame &&landmarks)
    : Frame<Motion>(id, timestamp, tWorldAgent, affine_brightness),
      keyframe_id_(keyframe_id),
      landmarks_(std::move(landmarks)) {}

template <energy::motion::Motion Motion>
void Keyframe<Motion>::points(std::map<std::string, storage::PointsStorage> &frame_points_storage) const {
  for (const auto &sensor_landmarks_iter : landmarks_) {
    buildPoints(this->tWorldAgent_, sensor_landmarks_iter.second, sensor_landmarks_iter.first,
                frame_points_storage["marginalized"], storage::PointStorageType::kMarginalized);
    buildPoints(this->tWorldAgent_, sensor_landmarks_iter.second, sensor_landmarks_iter.first,
                frame_points_storage["outlier"], storage::PointStorageType::kOutlier);
  }
}
template <energy::motion::Motion Motion>
size_t Keyframe<Motion>::keyframeId() const {
  return keyframe_id_;
}
template <energy::motion::Motion Motion>
Keyframe<Motion>::Keyframe(const proto::Keyframe &proto)
    : Frame<Motion>(proto.id(), time(std::chrono::high_resolution_clock::duration(proto.timestamp())), Motion(),
                    Eigen::Vector<Precision, 2>::Zero()),
      keyframe_id_(proto.keyframe_id()) {
  {
    CHECK_EQ(proto.t_world_agent_size(), Motion::num_parameters);
    Eigen::Vector<typename Motion::Scalar, Motion::num_parameters> parameters;

    for (int i = 0; i < Motion::num_parameters; i++) {
      parameters(i) = static_cast<typename Motion::Scalar>(proto.t_world_agent(i));
    }
    this->tWorldAgent_.setParameters(parameters);
  }

  {
    CHECK_EQ(proto.affine_brightness_size(), this->affine_brightness_.size());
    for (int i = 0; i < this->affine_brightness_.size(); i++) {
      this->affine_brightness_[i] = static_cast<Precision>(proto.affine_brightness(i));
    }
  }

  for (int i = 0; i < proto.tracking_frames_size(); i++) {
    const auto &tracking_frame = proto.tracking_frames(i);
    CHECK_EQ(tracking_frame.t_keyframe_agent_size(), Motion::Product::num_parameters);
    typename Motion::Product t_keyframe_tracking;

    Eigen::Vector<typename Motion::Scalar, Motion::Product::num_parameters> parameters;
    for (int j = 0; j < parameters.size(); j++) {
      parameters(j) = static_cast<typename Motion::Scalar>(tracking_frame.t_keyframe_agent(j));
    }
    t_keyframe_tracking.setParameters(parameters);

    Eigen::Vector<Precision, 2> affine_brightness;
    CHECK_EQ(tracking_frame.affine_brightness_size(), affine_brightness.size());
    for (int j = 0; j < affine_brightness.size(); j++) {
      affine_brightness[j] = static_cast<Precision>(tracking_frame.affine_brightness(j));
    }
    attachTrackingFrame(proto.id(), time(std::chrono::high_resolution_clock::duration(tracking_frame.timestamp())),
                        t_keyframe_tracking, affine_brightness);
    for (const auto &[sensor, buffer] : tracking_frame.image_buffer()) {
      std::vector<uchar> image_buffer(buffer.begin(), buffer.end());
      lastAttachedFrame().pushImage(sensor, std::move(image_buffer));
    }
  }

  for (int i = 0; i < proto.landmarks_size(); i++) {
    const auto &landmarks_frame = proto.landmarks(i);
    auto &landmarks = landmarks_[landmarks_frame.sensor_id()];
    landmarks.reserve(static_cast<size_t>(landmarks_frame.landmarks_size()));
    for (int j = 0; j < landmarks_frame.landmarks_size(); j++) {
      landmarks.emplace_back(landmarks::TrackingLandmark(landmarks_frame.landmarks(j)));
    }
  }
  for (const auto &[sensor, buffer] : proto.image_buffer()) {
    this->image_buffer_[sensor] = std::vector<uchar>(buffer.begin(), buffer.end());
  }
}

template <energy::motion::Motion Motion>
proto::Keyframe Keyframe<Motion>::proto() const {
  proto::Keyframe proto;
  proto.set_id(static_cast<unsigned int>(this->id_));
  proto.set_keyframe_id(static_cast<unsigned int>(this->keyframe_id_));
  proto.set_timestamp(static_cast<uint64_t>(this->timestamp_.time_since_epoch().count()));

  {
    auto parameters = this->tWorldAgent_.parameters();
    for (int i = 0; i < Motion::num_parameters; i++) {
      proto.add_t_world_agent(parameters(i));
    }
  }

  {
    for (int i = 0; i < this->affine_brightness_.size(); i++) {
      proto.add_affine_brightness(this->affine_brightness_[i]);
    }
  }

  for (const auto &frame : attached_frames_) {
    *proto.add_tracking_frames() = frame->proto();
  }

  for (auto const &[sensor_id, landmarks] : landmarks_) {
    proto::LandmarksFrame landmarks_frame;
    landmarks_frame.set_sensor_id(static_cast<unsigned int>(sensor_id));
    for (const auto &landmark : landmarks) {
      *landmarks_frame.add_landmarks() = landmark.proto();
    }
    *proto.add_landmarks() = landmarks_frame;
  }

  for (auto &image : this->image_buffer_) {
    std::string image_buffer_str(reinterpret_cast<const char *>(&image.second[0]), image.second.size());
    (*proto.mutable_image_buffer())[image.first] = image_buffer_str;
  }
  return proto;
}

template <energy::motion::Motion Motion>
const std::vector<landmarks::TrackingLandmark> &Keyframe<Motion>::landmarks(size_t sensor) const {
  return landmarks_.at(sensor);
}

template <energy::motion::Motion Motion>
void Keyframe<Motion>::attachTrackingFrame(size_t id, time timestamp, const typename Motion::Product &tKeyframeAgent,
                                           const Eigen::Vector<Precision, 2> &affine_brightness) {
  auto frame =
      std::make_unique<TrackingFrame<Motion>>(id, timestamp, this->tWorldAgent_, tKeyframeAgent, affine_brightness);
  attached_frames_.push_back(std::move(frame));
}

template <energy::motion::Motion Motion>
const std::vector<std::unique_ptr<TrackingFrame<Motion>>> &Keyframe<Motion>::attachedFrames() const {
  return attached_frames_;
}

template <energy::motion::Motion Motion>
TrackingFrame<Motion> &Keyframe<Motion>::lastAttachedFrame() {
  CHECK(!attached_frames_.empty());
  return *attached_frames_.back();
}

template <energy::motion::Motion Motion>
Keyframe<Motion>::~Keyframe() = default;

template class Keyframe<energy::motion::SE3<Precision>>;

}  // namespace track
}  // namespace dsopp
