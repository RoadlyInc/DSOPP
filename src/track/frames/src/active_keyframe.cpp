#include "track/frames/active_keyframe.hpp"

#include "common/patch/patch.hpp"
#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/camera_model/pinhole/simple_radial.hpp"

#include "features/camera/pattern_patch.hpp"
#include "features/camera/pixel_map.hpp"
#include "features/camera/tracking_feature.hpp"
#include "features/camera/tracking_features_frame.hpp"
#include "track/connections/frame_connection.hpp"
#include "track/landmarks/active_tracking_landmark.hpp"
#include "track/landmarks/immature_tracking_landmark.hpp"

#include <glog/logging.h>

namespace dsopp {
namespace track {
namespace {
template <int N, int C>
void separatePixelPatch(const Eigen::Vector<features::PixelInfo<C>, N> &pixel_patch,
                        Eigen::Matrix<Precision, N, C, PatchStorageOrder<C>> &patch,
                        Eigen::Matrix<Precision, 2, C> &gradient) {
  gradient.setZero();
  for (int i = 0; i < N; i++) {
    patch.row(i) = Eigen::Vector<Precision, C>(pixel_patch[i].intensity());
    gradient += pixel_patch[i].jacobian().transpose();
  }
}
}  // namespace
template <energy::motion::Motion Motion>
size_t ActiveKeyframe<Motion>::keyframeId() const {
  return keyframe_id_;
}

template <energy::motion::Motion Motion>
ActiveKeyframe<Motion>::ActiveKeyframe(size_t id, size_t keyframe_id, time timestamp, const Motion &tWorldAgent,
                                       const Precision exposure_time,
                                       const Eigen::Vector<Precision, 2> &affine_brightness)
    : Frame<Motion>(id, timestamp, tWorldAgent, exposure_time, affine_brightness),
      keyframe_id_(keyframe_id),
      is_marginalized_(false) {}

template <energy::motion::Motion Motion>
void ActiveKeyframe<Motion>::points(std::map<std::string, storage::PointsStorage> &frame_points_storage) const {
  for (const auto &sensor_landmarks_iter : active_landmarks_) {
    buildPoints(this->tWorldAgent_, sensor_landmarks_iter.second, sensor_landmarks_iter.first,
                frame_points_storage["active"], storage::PointStorageType::kActive);
    buildPoints(this->tWorldAgent_, sensor_landmarks_iter.second, sensor_landmarks_iter.first,
                frame_points_storage["marginalized"], storage::PointStorageType::kMarginalized);
    buildPoints(this->tWorldAgent_, sensor_landmarks_iter.second, sensor_landmarks_iter.first,
                frame_points_storage["outlier"], storage::PointStorageType::kOutlier);
  }
  for (const auto &sensor_landmarks_iter : immature_landmarks_) {
    buildPoints(this->tWorldAgent_, sensor_landmarks_iter.second, sensor_landmarks_iter.first,
                frame_points_storage["immature"]);
  }
}

template <energy::motion::Motion Motion>
void ActiveKeyframe<Motion>::setTWorldAgent(const Motion &tWorldAgent) {
  this->tWorldAgent_ = tWorldAgent;
  for (auto &frame : attached_frames_) {
    frame->onKeyframeTransformationChanged(tWorldAgent);
  }
}

template <energy::motion::Motion Motion>
void ActiveKeyframe<Motion>::setAffineBrightness(const Eigen::Vector<Precision, 2> &affine_brightness) {
  this->affine_brightness_ = affine_brightness;
}

template <energy::motion::Motion Motion>
void ActiveKeyframe<Motion>::attachTrackingFrame(
    size_t id, time timestamp, const typename Motion::Product &tKeyframeAgent, const Precision exposure_time,
    const Eigen::Vector<Precision, 2> &affine_brightness, Precision mean_square_optical_flow,
    Precision mean_square_optical_flow_without_rotation, Precision pose_rmse, bool valid) {
  auto frame = std::make_unique<SLAMInternalTrackingFrame<Motion>>(
      id, timestamp, this->tWorldAgent_, tKeyframeAgent, exposure_time, affine_brightness, mean_square_optical_flow,
      mean_square_optical_flow_without_rotation, pose_rmse, valid);
  attached_frames_.push_back(std::move(frame));
}

template <energy::motion::Motion Motion>
const std::vector<std::unique_ptr<SLAMInternalTrackingFrame<Motion>>> &ActiveKeyframe<Motion>::attachedFrames() const {
  return attached_frames_;
}

template <energy::motion::Motion Motion>
SLAMInternalTrackingFrame<Motion> &ActiveKeyframe<Motion>::lastAttachedFrame() {
  CHECK(!attached_frames_.empty());
  return *attached_frames_.back();
}

template <energy::motion::Motion Motion>
void ActiveKeyframe<Motion>::pushImmatureLandmarks(
    size_t sensor, const std::vector<std::pair<Eigen::Vector<Precision, 2>, Eigen::Vector<Precision, 3>>> &features) {
  std::vector<landmarks::ImmatureTrackingLandmark> immature_landmarks;
  std::vector<landmarks::ActiveTrackingLandmark> active_landmarks;
  for (const auto &[coordinates, direction] : features) {
    auto &image = (pyramids_.at(sensor))[0];
    Eigen::Vector<features::PixelInfo<1>, features::PatternPatch::N> pixel_patch;
    features::PatternPatch::getIntensities(coordinates, image, pixel_patch);
    Eigen::Vector<Precision, features::PatternPatch::N> patch;
    Eigen::Vector<Precision, 2> gradient;
    separatePixelPatch<features::PatternPatch::N, 1>(pixel_patch, patch, gradient);

    immature_landmarks.emplace_back(direction, coordinates, patch, gradient);
  }
  immature_landmarks_[sensor] = std::move(immature_landmarks);
  active_landmarks_[sensor] = std::move(active_landmarks);
}

template <energy::motion::Motion Motion>
void ActiveKeyframe<Motion>::pushSemanticsData(size_t sensor, std::unique_ptr<cv::Mat> &&semantics_data) {
  semantics_data_[sensor] = std::move(semantics_data);
}
template <energy::motion::Motion Motion>
const cv::Mat *ActiveKeyframe<Motion>::semanticsData(size_t sensor) const {
  if (semantics_data_.contains(sensor)) {
    return semantics_data_.at(sensor).get();
  }
  return nullptr;
}

template <energy::motion::Motion Motion>
const landmarks::ActiveTrackingLandmark &ActiveKeyframe<Motion>::getActiveLandmark(size_t sensor, size_t idx) const {
  CHECK_LT(idx, active_landmarks_.at(sensor).size());

  return active_landmarks_.at(sensor)[idx];
}

template <energy::motion::Motion Motion>
landmarks::ActiveTrackingLandmark &ActiveKeyframe<Motion>::getActiveLandmark(size_t sensor, size_t idx) {
  CHECK_LT(idx, active_landmarks_.at(sensor).size());

  return active_landmarks_.at(sensor)[idx];
}

template <energy::motion::Motion Motion>
const std::vector<landmarks::ActiveTrackingLandmark> &ActiveKeyframe<Motion>::activeLandmarks(size_t sensor) const {
  auto found = active_landmarks_.find(sensor);
  if (found == active_landmarks_.end()) {
    LOG(ERROR) << "`landmarks` from non existing sensor " << sensor;
  }

  return found->second;
}

template <energy::motion::Motion Motion>
const landmarks::ImmatureTrackingLandmark &ActiveKeyframe<Motion>::getImmatureLandmark(size_t sensor,
                                                                                       size_t idx) const {
  CHECK_LT(idx, immature_landmarks_.at(sensor).size());

  return immature_landmarks_.at(sensor)[idx];
}

template <energy::motion::Motion Motion>
landmarks::ImmatureTrackingLandmark &ActiveKeyframe<Motion>::getImmatureLandmark(size_t sensor, size_t idx) {
  CHECK_LT(idx, immature_landmarks_.at(sensor).size());

  return immature_landmarks_.at(sensor)[idx];
}

template <energy::motion::Motion Motion>
const std::vector<landmarks::ImmatureTrackingLandmark> &ActiveKeyframe<Motion>::immatureLandmarks(size_t sensor) const {
  auto found = immature_landmarks_.find(sensor);
  if (found == immature_landmarks_.end()) {
    LOG(ERROR) << "`landmarks` from non existing sensor";
  }

  return found->second;
}

template <energy::motion::Motion Motion>
void ActiveKeyframe<Motion>::marginalize() {
  is_marginalized_ = true;
  immature_landmarks_.clear();
  for (auto &[sensor_id, landmarks] : active_landmarks_) {
    for (auto &landmark : landmarks) {
      landmark.marginalize();
    }
  }
}

template <energy::motion::Motion Motion>
void ActiveKeyframe<Motion>::unloadResources() {
  CHECK(is_marginalized_);
  pyramids_.clear();
  pyramids_of_masks_.clear();
  semantics_data_.clear();
}

template <energy::motion::Motion Motion>
std::vector<size_t> ActiveKeyframe<Motion>::sensors() const {
  std::vector<size_t> sensors;
  for (const auto &[key, value] : active_landmarks_) {
    sensors.push_back(key);
  }
  return sensors;
}

template <energy::motion::Motion Motion>
bool ActiveKeyframe<Motion>::isMarginalized() const {
  return is_marginalized_;
}

template <energy::motion::Motion Motion>
std::map<size_t, size_t> ActiveKeyframe<Motion>::applyImmatureLandmarkActivationStatuses(
    const ImmatureLandmarkActivationStatusesPerSensor &statuses) {
  std::map<size_t, size_t> activated;
  for (auto &[sensor, immature_landmarks] : immature_landmarks_) {
    activated[sensor] = 0;
    assert(statuses.at(sensor).size() == immature_landmarks.size());
    for (size_t i = 0; i < immature_landmarks.size(); i++) {
      auto &immature_landmark = immature_landmarks[i];
      if (statuses.at(sensor)[i] == ImmatureLandmarkActivationStatus::kActivate) {
        landmarks::ActiveTrackingLandmark active_landmark(immature_landmark.direction(), immature_landmark.projection(),
                                                          immature_landmark.patch(), immature_landmark.idepth());
        if (semantics_data_.contains(sensor)) {
          if (auto semantics = semantics_data_[sensor].get()) {
            Eigen::Matrix<Precision, 2, Pattern::kSize> pattern;
            features::PatternPatch::shiftPattern(immature_landmark.projection(), pattern);
            for (int point_idx = 0; point_idx < Pattern::kSize; point_idx++) {
              uchar val = semantics->template at<uchar>(int(pattern(1, point_idx)), int(pattern(0, point_idx)));
              active_landmark.addSemanticTypeObservation(static_cast<size_t>(val));
            }
          }
        }
        active_landmarks_[sensor].push_back(std::move(active_landmark));
        activated[sensor]++;
        immature_landmark.setStatus(landmarks::ImmatureStatus::kDelete);
      } else if (statuses.at(sensor)[i] == ImmatureLandmarkActivationStatus::kDelete) {
        immature_landmark.setStatus(landmarks::ImmatureStatus::kDelete);
      }
    }
  }
  return activated;
}

template <energy::motion::Motion Motion>
void ActiveKeyframe<Motion>::pushPyramid(const size_t sensor, features::Pyramid &&pyramid) {
  pyramids_[sensor] = std::move(pyramid);
}

template <energy::motion::Motion Motion>
void ActiveKeyframe<Motion>::pushPyramidOfMasks(const size_t sensor,
                                                features::CameraFeatures::PyramidOfMasks &&pyramid_of_masks) {
  pyramids_of_masks_[sensor] = std::move(pyramid_of_masks);
}

template <energy::motion::Motion Motion>
const features::PixelMap<1> &ActiveKeyframe<Motion>::getLevel(const size_t sensor, size_t level) const {
  return (pyramids_.at(sensor))[level];
}

template <energy::motion::Motion Motion>
const sensors::calibration::CameraMask &ActiveKeyframe<Motion>::getMask(const size_t sensor, size_t level) const {
  return (*pyramids_of_masks_.at(sensor))[level];
}

template <energy::motion::Motion Motion>
const typename ActiveKeyframe<Motion>::PyramidsOfMasks &ActiveKeyframe<Motion>::pyramidsOfMasks() const {
  return pyramids_of_masks_;
}

template <energy::motion::Motion Motion>
const typename ActiveKeyframe<Motion>::Pyramids &ActiveKeyframe<Motion>::pyramids() const {
  return pyramids_;
}

template <energy::motion::Motion Motion>
const std::map<size_t, FrameConnection<typename Motion::Product> *> &ActiveKeyframe<Motion>::connections() const {
  return connections_;
}

template <energy::motion::Motion Motion>
FrameConnection<typename Motion::Product> &ActiveKeyframe<Motion>::getConnection(size_t target_keyframe_id) const {
  return *connections_.at(target_keyframe_id);
}

template <energy::motion::Motion Motion>
void ActiveKeyframe<Motion>::addConnection(size_t target_keyframe_id,
                                           FrameConnection<typename Motion::Product> *connection) {
  CHECK(target_keyframe_id == connection->targetKeyframeId() or
        target_keyframe_id == connection->referenceKeyframeId());
  connections_[target_keyframe_id] = connection;
}

template <energy::motion::Motion Motion>
ActiveKeyframe<Motion>::~ActiveKeyframe() = default;

template class ActiveKeyframe<energy::motion::SE3<Precision>>;

}  // namespace track
}  // namespace dsopp
