#include "track/frames/frame.hpp"

#include <opencv2/imgcodecs.hpp>

#include "common/settings.hpp"

#include "semantics/semantic_legend.hpp"
#include "track/landmarks/active_tracking_landmark.hpp"
#include "track/landmarks/immature_tracking_landmark.hpp"
#include "track/landmarks/tracking_landmark.hpp"

namespace dsopp {
namespace track {
namespace {

/**
 * Colors for semantic segmentation.
 */
const std::array<Eigen::Vector3<uint8_t>, 14> kSemanticColors = {
    Eigen::Vector3<uint8_t>(200, 200, 200), Eigen::Vector3<uint8_t>(71, 71, 107),
    Eigen::Vector3<uint8_t>(255, 77, 210),  Eigen::Vector3<uint8_t>(255, 255, 153),
    Eigen::Vector3<uint8_t>(255, 0, 0),     Eigen::Vector3<uint8_t>(51, 51, 153),
    Eigen::Vector3<uint8_t>(224, 224, 235), Eigen::Vector3<uint8_t>(255, 255, 0),
    Eigen::Vector3<uint8_t>(127, 127, 127), Eigen::Vector3<uint8_t>(0, 255, 0),
    Eigen::Vector3<uint8_t>(0, 0, 255),     Eigen::Vector3<uint8_t>(102, 51, 0),
    Eigen::Vector3<uint8_t>(51, 102, 153),  Eigen::Vector3<uint8_t>(139, 0, 255)};

bool validForVisualization(const landmarks::ActiveTrackingLandmark &landmark, storage::PointStorageType type,
                           const Precision min_relative_baseline = 0.1_p) {
  bool required = ((landmark.idepth()) != 0) && (landmark.relativeBaseline() > min_relative_baseline);
  bool outlier = type == storage::PointStorageType::kOutlier;
  bool nonoutlier = (landmark.isMarginalized() && type == storage::PointStorageType::kMarginalized) ||
                    (!landmark.isMarginalized() && type == storage::PointStorageType::kActive);

  return required && ((landmark.isOutlier() && outlier) || (!landmark.isOutlier() && nonoutlier));
}

bool validForVisualization(const landmarks::ImmatureTrackingLandmark &landmark, storage::PointStorageType) {
  return landmark.status() != landmarks::ImmatureStatus::kDelete;
}

bool validForVisualization(const landmarks::TrackingLandmark &landmark, storage::PointStorageType type,
                           const Precision min_relative_baseline = 0.1_p) {
  bool required = ((landmark.idepth()) != 0) && (landmark.relativeBaseline() > min_relative_baseline);

  bool outlier = type == storage::PointStorageType::kOutlier;
  bool nonoutlier = type == storage::PointStorageType::kMarginalized;

  return required && ((landmark.isOutlier() && outlier) || (!landmark.isOutlier() && nonoutlier));
}

// TODO: don't forget to pass legend in the landmark.semanticTypeId() when color will be moved
template <typename LandmarkType>
Eigen::Vector3<uint8_t> getColor(const LandmarkType &landmark, bool color_by_landmark_type, bool semantic_colors) {
  if (!color_by_landmark_type) {
    if (semantic_colors) {
      if constexpr (std::is_same<LandmarkType, landmarks::ImmatureTrackingLandmark>::value) {
        return Eigen::Vector3<uint8_t>::Constant(200);
      } else {
        return kSemanticColors[landmark.semanticTypeId() % kSemanticColors.size()];
      }
    } else {
      return Eigen::Vector3<uint8_t>::Constant(200);
    }
  }

  if constexpr (std::is_same<LandmarkType, landmarks::TrackingLandmark>::value) {
    return Eigen::Vector3<uint8_t>::Constant(200);
  }
  if constexpr (std::is_same<LandmarkType, landmarks::ImmatureTrackingLandmark>::value) {
    return Eigen::Vector3<uint8_t>(100, 100, 255);
  }
  if constexpr (std::is_same<LandmarkType, landmarks::ActiveTrackingLandmark>::value) {
    if (landmark.isOutlier()) {
      return Eigen::Vector3<uint8_t>(255, 0, 0);
    }
    if (landmark.isMarginalized()) {
      return Eigen::Vector3<uint8_t>(255, 100, 100);
    }
    return Eigen::Vector3<uint8_t>(100, 255, 100);
  }
}
}  // namespace
template <energy::motion::Motion Motion>
Frame<Motion>::Frame(size_t frame_id, time timestamp, const Motion &tWorldAgent, const Precision exposure_time,
                     const Eigen::Vector<Precision, 2> &affine_brightness)
    : id_(frame_id),
      timestamp_(timestamp),
      tWorldAgent_(tWorldAgent),
      exposure_time_(exposure_time),
      affine_brightness_(affine_brightness) {}

template <energy::motion::Motion Motion>
const Motion &Frame<Motion>::tWorldAgent() const {
  return tWorldAgent_;
}

template <energy::motion::Motion Motion>
time Frame<Motion>::timestamp() const {
  return timestamp_;
}

template <energy::motion::Motion Motion>
size_t Frame<Motion>::id() const {
  return id_;
}

template <energy::motion::Motion Motion>
Precision Frame<Motion>::exposureTime() const {
  return exposure_time_;
}

template <energy::motion::Motion Motion>
const Eigen::Vector<Precision, 2> &Frame<Motion>::affineBrightness() const {
  return affine_brightness_;
}

template <energy::motion::Motion Motion>
Frame<Motion>::~Frame() = default;

template <energy::motion::Motion Motion>
cv::Mat Frame<Motion>::image(const size_t sensor_id) const {
  if (!image_buffer_.contains(static_cast<unsigned int>(sensor_id))) return cv::Mat();

  auto &buffer = image_buffer_.at(static_cast<unsigned int>(sensor_id));

  if (buffer.empty()) return cv::Mat();
  return cv::imdecode(buffer, cv::IMREAD_UNCHANGED);
}

template <energy::motion::Motion Motion>
void Frame<Motion>::pushImage(const size_t sensor_id, const cv::Mat &image) {
  image_buffer_[static_cast<unsigned int>(sensor_id)] = {};

  if (!image.empty()) {
    const std::string kWriteExtension = ".jpg";
    const std::vector<int> kEncodeParameters = {cv::IMWRITE_JPEG_QUALITY, 100};

    cv::imencode(kWriteExtension, image, image_buffer_[static_cast<unsigned int>(sensor_id)], kEncodeParameters);
  }
}

template <energy::motion::Motion Motion>
void Frame<Motion>::pushImage(const size_t sensor_id, std::vector<uchar> &&image) {
  image_buffer_[static_cast<unsigned int>(sensor_id)] = std::move(image);
}

namespace {
template <typename LandmarkType>
Eigen::Vector<Precision, 3> getLandmarkCoordinate(const energy::motion::SE3<Precision> &,
                                                  const LandmarkType &landmark) {
  return landmark.direction() / landmark.idepth();
}

template <typename LandmarkType>
void fillRelativeBaselines(const std::vector<LandmarkType> &landmarks, std::vector<Precision> &relative_baselines,
                           storage::PointStorageType type) {
  if constexpr (!std::is_same_v<LandmarkType, landmarks::ImmatureTrackingLandmark>) {
    for (const auto &landmark : landmarks) {
      if (validForVisualization(landmark, type)) {
        relative_baselines.emplace_back(landmark.relativeBaseline());
      }
    }
  }
}

template <typename LandmarkType>
void fillIdepthVariances(const std::vector<LandmarkType> &landmarks, std::vector<Precision> &idepth_variances,
                         storage::PointStorageType type) {
  if constexpr (!std::is_same_v<LandmarkType, landmarks::ImmatureTrackingLandmark>) {
    for (const auto &landmark : landmarks) {
      if (validForVisualization(landmark, type)) {
        idepth_variances.emplace_back(landmark.idepthVariance());
      }
    }
  }
}
}  // namespace

template <typename PointData>
void insertInMap(std::map<size_t, std::vector<PointData>> &points, std::vector<PointData> &sensor_points,
                 size_t sensor_id) {
  if (!points.contains(sensor_id)) {
    points[sensor_id] = sensor_points;
  } else {
    points[sensor_id].insert(points[sensor_id].end(), sensor_points.begin(), sensor_points.end());
  }
}

template <typename LandmarkType, energy::motion::Motion Motion>
void buildPoints(const Motion &t_w_a, const std::vector<LandmarkType> &landmarks, const size_t sensor_id,
                 storage::PointsStorage &points_storage, storage::PointStorageType type) {
  std::vector<Eigen::Vector3<Precision>> sensor_points;
  std::vector<Eigen::Vector3<uint8_t>> sensor_colors;
  std::vector<Eigen::Vector3<uint8_t>> sensor_default_colors;
  std::vector<Eigen::Vector3<uint8_t>> sensor_semantics_colors;
  std::vector<Precision> sensor_idepth_variances;
  std::vector<Precision> sensor_relative_baselines;
  fillRelativeBaselines(landmarks, sensor_relative_baselines, type);
  fillIdepthVariances(landmarks, sensor_idepth_variances, type);

  for (const auto &landmark : landmarks) {
    if (validForVisualization(landmark, type)) {
      sensor_points.emplace_back(getLandmarkCoordinate(t_w_a, landmark));
      sensor_colors.emplace_back(getColor(landmark, true, false));
      sensor_default_colors.emplace_back(getColor(landmark, false, false));
      sensor_semantics_colors.emplace_back(getColor(landmark, false, true));
    }
  }
  insertInMap(points_storage.points, sensor_points, sensor_id);
  insertInMap(points_storage.colors, sensor_colors, sensor_id);
  insertInMap(points_storage.default_colors, sensor_default_colors, sensor_id);
  insertInMap(points_storage.semantics_colors, sensor_semantics_colors, sensor_id);
  insertInMap(points_storage.idepth_variances, sensor_idepth_variances, sensor_id);
  insertInMap(points_storage.relative_baselines, sensor_relative_baselines, sensor_id);
}

#define buildPointsInstantiation(Landmark, Motion)                                                         \
  template void buildPoints<Landmark, Motion>(const Motion &, const std::vector<Landmark> &, const size_t, \
                                              storage::PointsStorage &, storage::PointStorageType)
buildPointsInstantiation(landmarks::ActiveTrackingLandmark, energy::motion::SE3<Precision>);
buildPointsInstantiation(landmarks::TrackingLandmark, energy::motion::SE3<Precision>);
buildPointsInstantiation(landmarks::ImmatureTrackingLandmark, energy::motion::SE3<Precision>);
#undef buildPointsInstantiation

template class Frame<energy::motion::SE3<Precision>>;

}  // namespace track
}  // namespace dsopp
