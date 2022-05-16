#include "track/connections/frame_connection.hpp"

#include "energy/motion/se3_motion.hpp"

namespace dsopp {
namespace track {

template <energy::motion::MotionProduct MotionProduct>
FrameConnection<MotionProduct>::FrameConnection(size_t reference_keyframe_id, size_t target_keyframe_id)
    : reference_keyframe_id_(reference_keyframe_id), target_keyframe_id_(target_keyframe_id) {
  t_r_t_covariance_.setZero();
}

template <energy::motion::MotionProduct MotionProduct>
void FrameConnection<MotionProduct>::setCovariance(
    const Eigen::Matrix<Precision, MotionProduct::DoF, MotionProduct::DoF>& covariance) {
  t_r_t_covariance_ = covariance;
}

template <energy::motion::MotionProduct MotionProduct>
void FrameConnection<MotionProduct>::addSensorConnection(size_t reference_sensor_id, size_t target_sensor_id,
                                                         size_t reference_landmarks_size,
                                                         size_t target_landmarks_size) {
  auto& connections = reprojections_[std::make_pair(reference_sensor_id, target_sensor_id)];
  connections.first.resize(connections.first.size() + reference_landmarks_size, PointConnectionStatus::kOk);
  connections.second.resize(connections.second.size() + target_landmarks_size, PointConnectionStatus::kOk);
}

template <energy::motion::MotionProduct MotionProduct>
FrameConnection<MotionProduct>::FrameConnection(const proto::Connection& proto)
    : reference_keyframe_id_(proto.reference_keyframe_id()), target_keyframe_id_(proto.target_keyframe_id()) {
  t_r_t_covariance_ =
      Eigen::Map<const Eigen::Matrix<double, MotionProduct::DoF, MotionProduct::DoF>>(proto.covariance().data())
          .template cast<Precision>();
}

template <energy::motion::MotionProduct MotionProduct>
const Eigen::Matrix<Precision, MotionProduct::DoF, MotionProduct::DoF>& FrameConnection<MotionProduct>::covariance()
    const {
  return t_r_t_covariance_;
}

template <energy::motion::MotionProduct MotionProduct>
size_t FrameConnection<MotionProduct>::referenceKeyframeId() const {
  return reference_keyframe_id_;
}

template <energy::motion::MotionProduct MotionProduct>
size_t FrameConnection<MotionProduct>::targetKeyframeId() const {
  return target_keyframe_id_;
}

template <energy::motion::MotionProduct MotionProduct>
const typename FrameConnection<MotionProduct>::ReprojectionStatuses&
FrameConnection<MotionProduct>::referenceReprojectionStatuses(size_t reference_sensor_id,
                                                              size_t target_sensor_id) const {
  return reprojections_.at(std::pair(reference_sensor_id, target_sensor_id)).first;
}

template <energy::motion::MotionProduct MotionProduct>
const typename FrameConnection<MotionProduct>::ReprojectionStatuses&
FrameConnection<MotionProduct>::targetReprojectionStatuses(size_t reference_sensor_id, size_t target_sensor_id) const {
  return reprojections_.at(std::pair(reference_sensor_id, target_sensor_id)).second;
}

template <energy::motion::MotionProduct MotionProduct>
void FrameConnection<MotionProduct>::setReferenceReprojectionStatuses(
    size_t reference_sensor_id, size_t target_sensor_id, const FrameConnection::ReprojectionStatuses& new_statuses) {
  reprojections_.at(std::pair(reference_sensor_id, target_sensor_id)).first = new_statuses;
}

template <energy::motion::MotionProduct MotionProduct>
void FrameConnection<MotionProduct>::setTargetReprojectionStatuses(
    size_t reference_sensor_id, size_t target_sensor_id,
    const FrameConnection<MotionProduct>::ReprojectionStatuses& new_statuses) {
  reprojections_.at(std::pair(reference_sensor_id, target_sensor_id)).second = new_statuses;
}

template <energy::motion::MotionProduct MotionProduct>
std::vector<std::pair<size_t, size_t>> FrameConnection<MotionProduct>::getSensorPairs() const {
  std::vector<std::pair<size_t, size_t>> pairs;
  for (const auto& [pair, statuses] : reprojections_) {
    pairs.push_back(pair);
  }
  return pairs;
}

template <energy::motion::MotionProduct MotionProduct>
proto::Connection FrameConnection<MotionProduct>::proto() const {
  proto::Connection proto;
  proto.set_reference_keyframe_id(static_cast<unsigned int>(reference_keyframe_id_));
  proto.set_target_keyframe_id(static_cast<unsigned int>(target_keyframe_id_));
  for (int i = 0; i < MotionProduct::DoF * MotionProduct::DoF; ++i) {
    proto.add_covariance(t_r_t_covariance_.data()[i]);
  }
  return proto;
}

template class FrameConnection<energy::motion::SE3<Precision>>;

}  // namespace track
}  // namespace dsopp
