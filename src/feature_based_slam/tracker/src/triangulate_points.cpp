#include "feature_based_slam/tracker/triangulate_points.hpp"

#include <glog/logging.h>

#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/camera_model/pinhole/simple_radial.hpp"
#include "energy/epipolar_geometry/triangulation.hpp"
#include "feature_based_slam/features/correspondence.hpp"
#include "feature_based_slam/features/distinct_feature.hpp"
#include "feature_based_slam/features/distinct_features_frame.hpp"
#include "feature_based_slam/track/frame.hpp"
#include "feature_based_slam/track/landmarks.hpp"

namespace dsopp {
namespace feature_based_slam {
namespace tracker {

template <energy::model::Model Model>
void triangulatePoints(size_t reference_frame_id, size_t target_frame_id, track::Landmarks &landmarks,
                       const energy::motion::SE3<Precision> &t_w_r, const energy::motion::SE3<Precision> &t_w_t,
                       const Model &camera_model, const Precision triangulation_cos_threshold) {
  auto t_r_t = t_w_r.inverse() * t_w_t;
  Eigen::Matrix<Precision, 3, 4> t_r_t_matrix = t_r_t.matrix3x4();
  auto t_t_r = t_r_t.inverse();

  for (auto landmark_idx : landmarks.getLandmarksFromTwoFrames(reference_frame_id, target_frame_id)) {
    const auto &landmark = landmarks.landmarks()[landmark_idx];

    const auto *reference_point = landmark.projection(target_frame_id);
    const auto *target_point = landmark.projection(reference_frame_id);

    Eigen::Vector3<Precision> reference_ray, target_ray;

    if (camera_model.unproject(reference_point->coordinates(), reference_ray) and
        camera_model.unproject(target_point->coordinates(), target_ray)) {
      Eigen::Vector3<Precision> point3d =
          energy::epipolar_geometry::triangulation(target_ray, reference_ray, t_r_t_matrix);
      Precision reference_cos = reference_ray.normalized().dot((t_t_r * point3d).normalized());
      Precision target_cos = target_ray.normalized().dot(point3d.normalized());
      if (point3d(2) > 0 && reference_cos > triangulation_cos_threshold && target_cos > triangulation_cos_threshold) {
        landmarks.setPositionOfLandmark(landmark_idx, t_w_r * point3d);
      }
    }
  }
}

template <energy::model::Model Model>
void triangulatePoints(track::Landmarks &landmarks, std::deque<track::Frame> &frames, const Model &camera_model,
                       size_t min_number_of_projections, bool retriangulation) {
  CHECK_GE(min_number_of_projections, 2);
  for (size_t idx = 0; idx < landmarks.landmarks().size(); ++idx) {
    const auto &landmark = landmarks.landmarks()[idx];
    if (landmark.position() && !retriangulation) {
      continue;
    }
    std::vector<Eigen::Matrix<Precision, 3, 4>> transformations;
    std::vector<Eigen::Vector3<Precision>> bearing_vectors;
    for (const auto &frame : frames) {
      const auto *projection = landmark.projection(frame.frame_id);
      if (projection) {
        transformations.push_back(frame.t_world_agent.inverse().matrix3x4());
        Eigen::Vector3<Precision> ray;
        camera_model.unproject(projection->coordinates(), ray);
        bearing_vectors.push_back(ray.normalized());
      }
    }
    if (transformations.size() > min_number_of_projections) {
      landmarks.setPositionOfLandmark(idx, energy::epipolar_geometry::triangulation(bearing_vectors, transformations));
    }
  }
}

template void triangulatePoints<energy::model::PinholeCamera<Precision>>(
    size_t reference_frame_id, size_t target_frame_id, track::Landmarks &landmarks,
    const energy::motion::SE3<Precision> &t_w_r, const energy::motion::SE3<Precision> &t_w_t,
    const energy::model::PinholeCamera<Precision> &model, const Precision triangulation_cos_threshold);

template void triangulatePoints<energy::model::SimpleRadialCamera<Precision>>(
    size_t reference_frame_id, size_t target_frame_id, track::Landmarks &landmarks,
    const energy::motion::SE3<Precision> &t_w_r, const energy::motion::SE3<Precision> &t_w_t,
    const energy::model::SimpleRadialCamera<Precision> &model, const Precision triangulation_cos_threshold);

template void triangulatePoints<energy::model::PinholeCamera<Precision>>(
    track::Landmarks &landmarks, std::deque<track::Frame> &frames,
    const energy::model::PinholeCamera<Precision> &camera_model, size_t, bool);

template void triangulatePoints<energy::model::SimpleRadialCamera<Precision>>(
    track::Landmarks &landmarks, std::deque<track::Frame> &frames,
    const energy::model::SimpleRadialCamera<Precision> &camera_model, size_t, bool);
}  // namespace tracker
}  // namespace feature_based_slam
}  // namespace dsopp
