#include "track/frames/tracking_frame.hpp"

namespace dsopp {
namespace track {
template <energy::motion::Motion Motion>
TrackingFrame<Motion>::TrackingFrame(size_t id, time timestamp, const Motion& t_world_keyframe,
                                     const typename Motion::Product& t_keyframe_agent,
                                     const Eigen::Vector<Precision, 2>& affine_brightness)
    : Frame<Motion>(id, timestamp, t_world_keyframe * t_keyframe_agent, affine_brightness),
      tWorldKeyframe_(t_world_keyframe),
      tKeyframeAgent_(t_keyframe_agent) {}

template <energy::motion::Motion Motion>
proto::TrackingFrame TrackingFrame<Motion>::proto() const {
  proto::TrackingFrame tracking_frame;
  tracking_frame.set_timestamp(static_cast<uint64_t>(this->timestamp().time_since_epoch().count()));

  auto parameters = tKeyframeAgent_.parameters();
  for (int i = 0; i < Motion::Product::num_parameters; i++) {
    tracking_frame.add_t_keyframe_agent(parameters(i));
  }
  for (int i = 0; i < this->affine_brightness_.size(); i++) {
    tracking_frame.add_affine_brightness(this->affine_brightness_[i]);
  }

  for (auto& image : this->image_buffer_) {
    std::string image_buffer_str(reinterpret_cast<const char*>(&image.second[0]), image.second.size());
    (*tracking_frame.mutable_image_buffer())[image.first] = image_buffer_str;
  }

  return tracking_frame;
}

template <energy::motion::Motion Motion>
void TrackingFrame<Motion>::onKeyframeTransformationChanged(const Motion& t_world_keyframe) {
  tWorldKeyframe_ = t_world_keyframe;
  this->tWorldAgent_ = tWorldKeyframe_ * tKeyframeAgent_;
}

template <energy::motion::Motion Motion>
void TrackingFrame<Motion>::points(std::map<std::string, storage::PointsStorage>&) const {}

template <energy::motion::Motion Motion>
const Motion& TrackingFrame<Motion>::tWorldKeyframe() const {
  return tWorldKeyframe_;
}

template <energy::motion::Motion Motion>
const typename Motion::Product& TrackingFrame<Motion>::tKeyframeAgent() const {
  return tKeyframeAgent_;
}

template class TrackingFrame<energy::motion::SE3<Precision>>;

}  // namespace track
}  // namespace dsopp
