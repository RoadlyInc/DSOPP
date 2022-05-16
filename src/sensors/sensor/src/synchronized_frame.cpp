#include "sensor/synchronized_frame.hpp"

#include <glog/logging.h>
#include "features/camera/camera_features.hpp"

namespace dsopp {
namespace sensors {
void SynchronizedFrame::addCameraFeatures(size_t id, std::unique_ptr<features::CameraFeatures>&& frame) {
  CHECK(timestamp_ == frame->timestamp());
  camera_frames_.emplace(id, std::move(frame));
}
void SynchronizedFrame::addGnssFeatures(size_t, std::unique_ptr<features::GnssFeatures>&&) {
  LOG(WARNING) << "contact Roadly INC for this functionality ";
}
void SynchronizedFrame::addIMUFeatures(size_t, std::unique_ptr<features::IMUFeatures>&&) {
  LOG(WARNING) << "contact Roadly INC for this functionality ";
}

void SynchronizedFrame::setTWorldLocal(const SE3d& t_world_local) { t_world_local_ = t_world_local; }
const std::map<size_t, std::unique_ptr<features::CameraFeatures>>& SynchronizedFrame::cameraFeatures() const {
  return camera_frames_;
}
const SynchronizedFrame::SE3d& SynchronizedFrame::tWorldLocal() const { return t_world_local_; }

int SynchronizedFrame::id() const { return frame_id_; }
time SynchronizedFrame::timestamp() const { return timestamp_; }

SynchronizedFrame::SynchronizedFrame(int id, time timestamp) : frame_id_(id), timestamp_(timestamp) {}

SynchronizedFrame::~SynchronizedFrame() = default;
}  // namespace sensors
}  // namespace dsopp
