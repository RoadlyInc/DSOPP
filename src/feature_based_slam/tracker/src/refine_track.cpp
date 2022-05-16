#include "feature_based_slam/tracker/refine_track.hpp"

#include <glog/logging.h>

#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/camera_model/pinhole/simple_radial.hpp"
#include "feature_based_slam/features/distinct_feature.hpp"
#include "feature_based_slam/track/frame.hpp"
#include "feature_based_slam/track/landmarks.hpp"

namespace dsopp {
namespace feature_based_slam {
namespace tracker {

template <energy::model::Model Model, bool OPTIMIZE_CAMERA>
void optimizeTransformations(track::Landmarks &landmarks, const size_t number_of_threads,
                             const Eigen::Vector<Precision, 2> &image_size,
                             typename traits<OPTIMIZE_CAMERA, Model>::type &camera_intrinsics,
                             std::deque<track::Frame> &initialization_frames, const Precision reprojection_threshold,
                             const size_t number_of_frames_to_optimize, const Precision max_solver_time,
                             ParameterParameterization parameter_parameterization) {
  using LocalFrame = energy::problem::geometric_bundle_adjustment::LocalFrame;
  std::vector<LocalFrame> frames;

  size_t number_of_frames = std::min(initialization_frames.size(), number_of_frames_to_optimize);
  size_t start = initialization_frames.size() - number_of_frames;
  size_t end = start + number_of_frames;

  std::vector<Eigen::Vector3<double>> points_3d_double;
  std::unordered_map<size_t, size_t> matching_indices;
  for (size_t l_i = 0; l_i < landmarks.landmarks().size(); l_i++) {
    const auto &landmark = landmarks.landmarks()[l_i];
    bool exists_in_window = false;
    for (size_t i = start; i < end; i++) {
      auto &frame = initialization_frames[i];
      if (landmark.position() && landmark.projection(frame.frame_id)) {
        exists_in_window = true;
      }
    }

    if (exists_in_window) {
      points_3d_double.push_back(landmark.position()->cast<double>());
      matching_indices[l_i] = points_3d_double.size() - 1;
    }
  }

  for (size_t i = start; i < end; i++) {
    auto &frame = initialization_frames[i];
    if (!frame.initialized) continue;

    auto t_agent_world = frame.t_world_agent.template cast<double>().inverse();
    LocalFrame local_frame(i, t_agent_world);

    for (const auto &landmark_idx : landmarks.getLandmarksFromFrame(frame.frame_id)) {
      const auto &landmark = landmarks.landmarks()[landmark_idx];
      if (!landmark.position() || !matching_indices.contains(landmark_idx)) continue;
      local_frame.pushObservation(landmark.projection(frame.frame_id)->coordinates().template cast<double>(),
                                  points_3d_double[matching_indices[landmark_idx]]);
    }

    frames.emplace_back(local_frame);
  }

  energy::problem::geometric_bundle_adjustment::CeresGeometricBundleAdjustmentSolver<Model> solver(
      image_size, camera_intrinsics, static_cast<int>(number_of_threads), reprojection_threshold, max_solver_time);
  if (!parameter_parameterization.fix_focal || !parameter_parameterization.fix_center ||
      !parameter_parameterization.fix_model_specific) {
    CHECK(OPTIMIZE_CAMERA) << "OPTIMIZE_CAMERA parameters have to be `true`";
  }
  solver.solve(frames, parameter_parameterization);

  for (auto &frame : frames) {
    initialization_frames[frame.frameId()].t_world_agent = frame.t_agent_world.template cast<Precision>().inverse();
  }

  for (auto idx : matching_indices) {
    landmarks.setPositionOfLandmark(idx.first, points_3d_double[idx.second].cast<Precision>());
  }

  if constexpr (OPTIMIZE_CAMERA) {
    camera_intrinsics = solver.camera_intrinsics().template cast<Precision>();
  }
}

template void optimizeTransformations<energy::model::PinholeCamera<Precision>, false>(
    track::Landmarks &, const size_t, const Eigen::Vector<Precision, 2> &,
    const Eigen::Vector<Precision, energy::model::PinholeCamera<Precision>::DoF> &, std::deque<track::Frame> &,
    const Precision, const size_t, const Precision, ParameterParameterization);

template void optimizeTransformations<energy::model::SimpleRadialCamera<Precision>, false>(
    track::Landmarks &, const size_t, const Eigen::Vector<Precision, 2> &,
    const Eigen::Vector<Precision, energy::model::SimpleRadialCamera<Precision>::DoF> &, std::deque<track::Frame> &,
    const Precision, const size_t, const Precision, ParameterParameterization);

template void optimizeTransformations<energy::model::PinholeCamera<Precision>, true>(
    track::Landmarks &, const size_t, const Eigen::Vector<Precision, 2> &,
    Eigen::Vector<Precision, energy::model::PinholeCamera<Precision>::DoF> &, std::deque<track::Frame> &,
    const Precision, const size_t, const Precision, ParameterParameterization);

template void optimizeTransformations<energy::model::SimpleRadialCamera<Precision>, true>(
    track::Landmarks &, const size_t, const Eigen::Vector<Precision, 2> &,
    Eigen::Vector<Precision, energy::model::SimpleRadialCamera<Precision>::DoF> &, std::deque<track::Frame> &,
    const Precision, const size_t, const Precision, ParameterParameterization);
}  // namespace tracker
}  // namespace feature_based_slam
}  // namespace dsopp
