#include "tracker/create_depth_maps.hpp"

#include "common/settings.hpp"
#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/camera_model/pinhole/simple_radial.hpp"
#include "energy/motion/se3_motion.hpp"

#include "energy/projector/camera_reproject.hpp"
#include "features/camera/pixel_data_frame.hpp"
#include "features/camera/pixel_map.hpp"

#include "track/connections/frame_connection.hpp"
#include "track/landmarks/active_tracking_landmark.hpp"

namespace dsopp::tracker {

namespace {
template <energy::motion::Motion Motion, energy::model::Model Model>
void fillFineDepthMap(const std::deque<track::ActiveKeyframe<Motion> *> &frames, const size_t sensor,
                      std::vector<energy::problem::DepthMap> &depth_maps,
                      const sensors::calibration::CameraCalibration &calibration) {
  const Precision kEps = 1e-12_p;
  // Need for the numeric stability
  const Precision kVariationScale = 1e-3_p;
  // Fill out first pyramid level
  auto model = calibration.cameraModel<Model>();
  for (size_t i = 0; i < frames.size() - 1; i++) {
    const auto &frame = frames[i];
    auto t_t_r = frames.back()->tWorldAgent().inverse() * frame->tWorldAgent();
    const auto &connections =
        frame->getConnection(frames.back()->keyframeId()).referenceReprojectionStatuses(sensor, sensor);
    energy::reprojection::ArrayReprojector<Precision, Model, typename Motion::Product> reprojector(*model, t_t_r);
    const auto &landmarks = frame->activeLandmarks(sensor);
    auto connections_status_iter = connections.cbegin();
    for (auto landmark_iter = landmarks.cbegin(); connections_status_iter != connections.cend();
         connections_status_iter++, landmark_iter++) {
      if (*connections_status_iter != track::PointConnectionStatus::kOk) continue;
      const auto &landmark = *landmark_iter;
      if (landmark.isOutlier() || landmark.isMarginalized()) continue;

      Eigen::Vector2<Precision> point_2d;

      if (!reprojector.reproject(landmark.projection(), landmark.idepth(), point_2d)) {
        continue;
      }

      Eigen::Vector2i idx = round(point_2d.array()).cast<int>();

      // TODO: add getDepthScale to the reprojector
      Precision depth_scale = model->getDepthScale(landmark.direction(), landmark.idepth(), t_t_r);

      Precision weight = std::sqrt(kVariationScale / (landmark.idepthVariance() + kEps));
      depth_maps[0].map(idx.x(), idx.y()).idepth += landmark.idepth() / depth_scale * weight;
      depth_maps[0].map(idx.x(), idx.y()).weight += weight;
    }
  }
}

}  // namespace

void initDepthMaps(const features::Pyramid &pyramid, std::vector<energy::problem::DepthMap> &idepth_maps) {
  idepth_maps.reserve(pyramid.size());
  for (auto &map : pyramid) {
    idepth_maps.emplace_back(energy::problem::DepthMap(map.width(), map.height()));
  }
}

void fillCoarseDepthMaps(std::vector<energy::problem::DepthMap> &depth_maps) {
  for (size_t lvl = 1; lvl < depth_maps.size(); lvl++) {
    size_t lvl_up = lvl - 1;

    auto &depth_map = depth_maps[lvl].map;
    auto &depth_map_up = depth_maps[lvl_up].map;

    long width = depth_map.rows(), height = depth_map.cols();

    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        depth_map(x, y) = depth_map_up(2 * x, 2 * y) + depth_map_up(2 * x + 1, 2 * y) + depth_map_up(2 * x, 2 * y + 1) +
                          depth_map_up(2 * x + 1, 2 * y + 1);
      }
    }
  }
}

void dilateDepthMaps(std::vector<energy::problem::DepthMap> &depth_maps) {
  for (size_t lvl = 0; lvl < depth_maps.size(); lvl++) {
    auto &depth_map = depth_maps[lvl].map;
    long width = depth_map.rows(), height = depth_map.cols();
    Eigen::MatrixX<Precision> weight_map_backup = Eigen::MatrixX<Precision>::Zero(width, height);
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        weight_map_backup(x, y) = depth_map(x, y).weight;
      }
    }
    for (int y = 1; y < height - 1; y++) {
      for (int x = 1; x < width - 1; x++) {
        if (weight_map_backup(x, y) <= 0) {
          Precision sum = 0, num = 0, numn = 0;
          Eigen::Matrix<int, 4, 2> offset;
          if (lvl > 1) {
            offset << 1, 0, -1, 0, 0, 1, 0, -1;
          } else {
            offset << 1, 1, -1, -1, 1, -1, -1, 1;
          }
          for (int i = 0; i < 4; i++) {
            Eigen::Vector2i coords = Eigen::Vector2i(x, y) + offset.row(i).transpose();
            if (weight_map_backup(coords(0), coords(1)) > 0) {
              sum += depth_map(coords(0), coords(1)).idepth;
              num += weight_map_backup(coords(0), coords(1));
              numn++;
            }
          }
          if (numn > 0) {
            depth_map(x, y).idepth = sum / numn;
            depth_map(x, y).weight = num / numn;
          }
        }
      }
    }
  }
}

template <energy::motion::Motion Motion, energy::model::Model Model>
std::map<size_t, std::vector<energy::problem::DepthMap>> createReferenceDepthMaps(
    const std::deque<track::ActiveKeyframe<Motion> *> &frames,
    const sensors::calibration::CameraCalibration &calibration) {
  auto &target = *frames.back();
  std::map<size_t, std::vector<energy::problem::DepthMap>> reference_depth_maps;

  if (frames.empty()) {
    return reference_depth_maps;
  }

  for (const auto &[sensor, pyramid] : target.pyramids()) {
    std::vector<energy::problem::DepthMap> depth_maps;

    initDepthMaps(pyramid, depth_maps);
    fillFineDepthMap<Motion, Model>(frames, sensor, depth_maps, calibration);
    fillCoarseDepthMaps(depth_maps);
    dilateDepthMaps(depth_maps);

    reference_depth_maps[sensor] = std::move(depth_maps);
  }

  return reference_depth_maps;
}

#define createReferenceDepthMapsInstantation(M, Model)                      \
  template std::map<size_t, std::vector<energy::problem::DepthMap>>         \
  createReferenceDepthMaps<energy::motion::M, Model>(                       \
      const std::deque<track::ActiveKeyframe<energy::motion::M> *> &frames, \
      const sensors::calibration::CameraCalibration &calibration)

createReferenceDepthMapsInstantation(SE3<Precision>, energy::model::PinholeCamera<Precision>);
createReferenceDepthMapsInstantation(SE3<Precision>, energy::model::SimpleRadialCamera<Precision>);

#undef createReferenceDepthMapsInstantation
}  // namespace dsopp::tracker
