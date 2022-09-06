#include "energy/problems/photometric_bundle_adjustment/photometric_bundle_adjustment.hpp"

#include <tbb/parallel_for.h>

#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/camera_model/pinhole/simple_radial.hpp"

#include "energy/problems/photometric_bundle_adjustment/bundle_adjustment_photometric_evaluation_callback.hpp"
#include "energy/problems/photometric_bundle_adjustment/first_estimate_jacobians.hpp"
#include "energy/problems/photometric_bundle_adjustment/local_frame.hpp"
#include "energy/projector/camera_reproject.hpp"
#include "features/camera/ceres_grid.hpp"
#include "features/camera/pixel_map.hpp"

#include "track/landmarks/active_tracking_landmark.hpp"

namespace dsopp {
namespace energy {
namespace problem {

template <typename Scalar, energy::motion::Motion Motion, model::Model Model, int PatternSize,
          template <int> typename Grid2D, bool OPTIMIZE_POSES, bool OPTIMIZE_IDEPTHS, bool FIRST_ESTIMATE_JACOBIANS,
          int C>

LocalFrame<Scalar, Motion, Model, PatternSize, Grid2D, C>
    *PhotometricBundleAdjustment<Scalar, Motion, Model, PatternSize, Grid2D, OPTIMIZE_POSES, OPTIMIZE_IDEPTHS,
                                 FIRST_ESTIMATE_JACOBIANS, C>::getLocalFrame(const int id) const {
  for (auto &local_frame_iter : frames_) {
    if (local_frame_iter->id == id) {
      return local_frame_iter.get();
    }
  }
  return nullptr;
}
template <typename Scalar, energy::motion::Motion Motion, model::Model Model, int PatternSize,
          template <int> typename Grid2D, bool OPTIMIZE_POSES, bool OPTIMIZE_IDEPTHS, bool FIRST_ESTIMATE_JACOBIANS,
          int C>
LocalFrame<Scalar, Motion, Model, PatternSize, Grid2D, C>
    *PhotometricBundleAdjustment<Scalar, Motion, Model, PatternSize, Grid2D, OPTIMIZE_POSES, OPTIMIZE_IDEPTHS,
                                 FIRST_ESTIMATE_JACOBIANS, C>::getLocalFrame(time timestamp) const {
  for (auto &local_frame_iter : frames_) {
    if (local_frame_iter->timestamp == timestamp) {
      return local_frame_iter.get();
    }
  }
  return nullptr;
}
template <typename Scalar, energy::motion::Motion Motion, model::Model Model, int PatternSize,
          template <int> typename Grid2D, bool OPTIMIZE_POSES, bool OPTIMIZE_IDEPTHS, bool FIRST_ESTIMATE_JACOBIANS,
          int C>
PhotometricBundleAdjustment<Scalar, Motion, Model, PatternSize, Grid2D, OPTIMIZE_POSES, OPTIMIZE_IDEPTHS,
                            FIRST_ESTIMATE_JACOBIANS, C>::PhotometricBundleAdjustment(bool estimate_uncertainty)
    : estimate_uncertainty_(estimate_uncertainty) {}

template <typename Scalar, energy::motion::Motion Motion, model::Model Model, int PatternSize,
          template <int> typename Grid2D, bool OPTIMIZE_POSES, bool OPTIMIZE_IDEPTHS, bool FIRST_ESTIMATE_JACOBIANS,
          int C>
void PhotometricBundleAdjustment<Scalar, Motion, Model, PatternSize, Grid2D, OPTIMIZE_POSES, OPTIMIZE_IDEPTHS,
                                 FIRST_ESTIMATE_JACOBIANS,
                                 C>::pushFrame(time timestamp, const Motion &t_world_agent, const Pyramids &pyramids,
                                               const std::map<size_t, const sensors::calibration::CameraMask &> &masks,
                                               const std::map<size_t, std::vector<DepthMap>> &depths_maps,
                                               const Precision exposure_time,
                                               const Eigen::Vector2<Precision> &affine_brightness, size_t level,
                                               const Model &model, FrameParameterization frame_parameterization) {
  if (not frames_.empty() and frames_.back()->timestamp > timestamp) {
    LOG(ERROR) << "Frames must be processed in ascending order of time";
    return;
  }

  frames_.push_back(std::make_unique<LocalFrame<Scalar, Motion, Model, PatternSize, Grid2D, C>>(
      timestamp, t_world_agent, pyramids, masks, depths_maps, static_cast<Scalar>(exposure_time),
      affine_brightness.template cast<Scalar>(), level, model, frame_parameterization));
}
template <typename Scalar, energy::motion::Motion Motion, model::Model Model, int PatternSize,
          template <int> typename Grid2D, bool OPTIMIZE_POSES, bool OPTIMIZE_IDEPTHS, bool FIRST_ESTIMATE_JACOBIANS,
          int C>
void PhotometricBundleAdjustment<
    Scalar, Motion, Model, PatternSize, Grid2D, OPTIMIZE_POSES, OPTIMIZE_IDEPTHS, FIRST_ESTIMATE_JACOBIANS,
    C>::pushFrame(time timestamp, const Motion &t_world_agent, const Pyramids &pyramids,
                  const std::map<size_t, const sensors::calibration::CameraMask &> &masks,
                  const std::map<size_t, std::vector<track::landmarks::TrackingLandmark>> &tracking_landmarks,
                  const Precision exposure_time, const Eigen::Vector2<Precision> &affine_brightness, const Model &model,
                  size_t level, Precision level_resize_ratio, FrameParameterization frame_parameterization) {
  if (not frames_.empty() and frames_.back()->timestamp > timestamp) {
    LOG(ERROR) << "Frames must be processed in ascending order of time";
    return;
  }
  frames_.push_back(std::make_unique<LocalFrame<Scalar, Motion, Model, PatternSize, Grid2D, C>>(
      timestamp, t_world_agent, pyramids, masks, tracking_landmarks, static_cast<Scalar>(exposure_time),
      affine_brightness.template cast<Scalar>(), model, level, level_resize_ratio, frame_parameterization));
}

template <typename Scalar, energy::motion::Motion Motion, model::Model Model, int PatternSize,
          template <int> typename Grid2D, bool OPTIMIZE_POSES, bool OPTIMIZE_IDEPTHS, bool FIRST_ESTIMATE_JACOBIANS,
          int C>
void PhotometricBundleAdjustment<Scalar, Motion, Model, PatternSize, Grid2D, OPTIMIZE_POSES, OPTIMIZE_IDEPTHS,
                                 FIRST_ESTIMATE_JACOBIANS, C>::pushFrame(const track::ActiveKeyframe<Motion> &frame,
                                                                         size_t level, const Model &model,
                                                                         FrameParameterization frame_parameterization) {
  CHECK(frames_.empty() or frames_.back()->timestamp < frame.timestamp())
      << "Frames must be processed in ascending order of time";

  frames_.push_back(std::make_unique<LocalFrame<Scalar, Motion, Model, PatternSize, Grid2D, C>>(
      frame, level, model, frame_parameterization));
  LocalFrame<Scalar, Motion, Model, PatternSize, Grid2D, C> *local_frame = frames_.back().get();
  int id = static_cast<int>(frame.keyframeId());
  // add reprojections between new frame and all previous
  for (const auto &[reference_id, connection] : frame.connections()) {
    LocalFrame<Scalar, Motion, Model, PatternSize, Grid2D, C> *reference_frame =
        getLocalFrame(static_cast<int>(reference_id));
    if (!reference_frame || reference_frame->is_marginalized) continue;
    for (const auto &pair : connection->getSensorPairs()) {
      auto &reference_residuals = reference_frame->residuals[pair][id];
      for (auto status : connection->referenceReprojectionStatuses(pair.first, pair.second)) {
        reference_residuals.push_back(ResidualPoint<Scalar, Motion, PatternSize, C>(status));
      }
      auto &target_residuals = local_frame->residuals[pair][reference_frame->id];
      for (auto status : connection->targetReprojectionStatuses(pair.first, pair.second)) {
        target_residuals.push_back(ResidualPoint<Scalar, Motion, PatternSize, C>(status));
      }
    }
  }
}

template <typename Scalar, energy::motion::Motion Motion, model::Model Model, int PatternSize,
          template <int> typename Grid2D, bool OPTIMIZE_POSES, bool OPTIMIZE_IDEPTHS, bool FIRST_ESTIMATE_JACOBIANS,
          int C>
void PhotometricBundleAdjustment<Scalar, Motion, Model, PatternSize, Grid2D, OPTIMIZE_POSES, OPTIMIZE_IDEPTHS,
                                 FIRST_ESTIMATE_JACOBIANS,
                                 C>::pushFrame(time timestamp, const Motion &t_world_agent_init,
                                               const Pyramids &pyramids,
                                               const std::map<size_t, const sensors::calibration::CameraMask &> &masks,
                                               const Precision exposure_time,
                                               const Eigen::Vector2<Precision> &affine_brightness, size_t level,
                                               const Model &model, FrameParameterization frame_parameterization) {
  if (not frames_.empty() and frames_.back()->timestamp > timestamp) {
    LOG(ERROR) << "Frames must be processed in ascending order of time";
    return;
  }
  for (auto &frame : frames_) {
    for (auto &[sensor, landmarks] : frame->active_landmarks) {
      auto &residuals = frame->residuals[std::make_pair(
          sensor, sensor)][LocalFrame<Scalar, Motion, Model, PatternSize, Grid2D, C>::kFrontendTargetFrameId];
      for (size_t landmark_id = 0; landmark_id < landmarks.size(); landmark_id++) {
        residuals.push_back(track::PointConnectionStatus::kOk);
      }
    }
  }

  frames_.push_back(std::make_unique<LocalFrame<Scalar, Motion, Model, PatternSize, Grid2D, C>>(
      timestamp, t_world_agent_init, pyramids, masks, static_cast<Scalar>(exposure_time),
      affine_brightness.template cast<Scalar>(), false, level, model, frame_parameterization));
}

template <typename Scalar, energy::motion::Motion Motion, model::Model Model, int PatternSize,
          template <int> typename Grid2D, bool OPTIMIZE_POSES, bool OPTIMIZE_IDEPTHS, bool FIRST_ESTIMATE_JACOBIANS,
          int C>
Motion PhotometricBundleAdjustment<Scalar, Motion, Model, PatternSize, Grid2D, OPTIMIZE_POSES, OPTIMIZE_IDEPTHS,
                                   FIRST_ESTIMATE_JACOBIANS, C>::getPose(time timestamp) const {
  auto local_frame = getLocalFrame(timestamp);
  CHECK(local_frame != nullptr);
  return local_frame->tWorldAgent().template cast<Precision>();
}

template <typename Scalar, energy::motion::Motion Motion, model::Model Model, int PatternSize,
          template <int> typename Grid2D, bool OPTIMIZE_POSES, bool OPTIMIZE_IDEPTHS, bool FIRST_ESTIMATE_JACOBIANS,
          int C>
Eigen::Vector2<Precision>
PhotometricBundleAdjustment<Scalar, Motion, Model, PatternSize, Grid2D, OPTIMIZE_POSES, OPTIMIZE_IDEPTHS,
                            FIRST_ESTIMATE_JACOBIANS, C>::getAffineBrightness(time timestamp) const {
  auto local_frame = getLocalFrame(timestamp);
  if (not local_frame) {
    return Eigen::Vector2<Precision>::Zero();
  }
  return local_frame->affineBrightness().template cast<Precision>();
}

template <typename Scalar, energy::motion::Motion Motion, model::Model Model, int PatternSize,
          template <int> typename Grid2D, bool OPTIMIZE_POSES, bool OPTIMIZE_IDEPTHS, bool FIRST_ESTIMATE_JACOBIANS,
          int C>
void PhotometricBundleAdjustment<Scalar, Motion, Model, PatternSize, Grid2D, OPTIMIZE_POSES, OPTIMIZE_IDEPTHS,
                                 FIRST_ESTIMATE_JACOBIANS, C>::updateFrame(track::ActiveKeyframe<Motion> &frame) {
  const double kIdepthEps = 1e-8;
  auto local_frame = getLocalFrame(frame.timestamp());
  CHECK(local_frame) << "Cannot update frame, there is no local copy in the solver";
  for (const auto &[sensors, residuals] : local_frame->residuals) {
    for (const auto &[target_id, point_residuals] : residuals) {
      if (target_id == LocalFrame<Scalar, Motion, Model, PatternSize, Grid2D, C>::kFrontendReferenceFrameId or
          target_id == LocalFrame<Scalar, Motion, Model, PatternSize, Grid2D, C>::kFrontendTargetFrameId or
          point_residuals.empty()) {
        continue;
      }
      auto target_frame = getLocalFrame(target_id);
      if (!target_frame) {
        continue;
      }
      std::vector<track::PointConnectionStatus> point_statuses(point_residuals.size());
      for (size_t i = 0; i < point_residuals.size(); i++) {
        point_statuses[i] = point_residuals[i].connection_status;
      }
      auto &connection = frame.getConnection(static_cast<size_t>(target_id));

      if (estimate_uncertainty_) {
        connection.setCovariance(local_frame->covariance_matrices.at(target_id).template cast<Precision>());
      }

      if (point_residuals.empty()) continue;

      if (target_id > static_cast<int>(frame.keyframeId())) {
        frame.getConnection(static_cast<size_t>(target_id))
            .setReferenceReprojectionStatuses(sensors.first, sensors.second, point_statuses);
      } else {
        frame.getConnection(static_cast<size_t>(target_id))
            .setTargetReprojectionStatuses(sensors.first, sensors.second, point_statuses);
      }
    }
  }
  if constexpr (OPTIMIZE_POSES) {
    frame.setTWorldAgent(local_frame->tWorldAgent().template cast<Precision>());
    frame.setAffineBrightness(local_frame->affineBrightness().template cast<Precision>());
  }
  if constexpr (OPTIMIZE_IDEPTHS) {
    for (auto &local_landmarks_iter : local_frame->active_landmarks) {
      const auto &sensor = local_landmarks_iter.first;
      auto &local_landmarks = local_landmarks_iter.second;
      for (size_t idx = 0; idx < frame.activeLandmarks(sensor).size(); idx++) {
        auto &landmark = frame.getActiveLandmark(sensor, idx);
        if (local_landmarks[idx].is_outlier) {
          landmark.markOutlier();
        }
        if (not local_landmarks[idx].is_marginalized) {
          if (std::abs(local_landmarks[idx].idepth) < kIdepthEps) {
            landmark.setIdepth(0);
          } else if (local_landmarks[idx].idepth < 0) {
            landmark.markOutlier();
          } else {
            landmark.setIdepth(static_cast<Precision>(local_landmarks[idx].idepth));
          }
          if (estimate_uncertainty_) {
            /**
             * In case of identity embedder new jacobian J_new contains C copies of old jacobian J_old. Hence, new
             * hessian H_new = J_new' * J_new = C * H_old = C * J_old' * J_old because H_new_ij = (J_new_:i,
             * J_new_:j) = C * (J_old_:i, J_old_:j) = C * H_old_ij (here we assume that pixel intensities are
             * independent and equally normally distributed, so covariance matrix is diagonal and proportional
             * to identity matrix).
             *
             * TODO: think about case of GN-Net embedder
             *
             * Inverse depth variation prevails over pose variation, so we can use this result approximately.
             */
            landmark.setIdepthVariance(static_cast<Precision>(local_landmarks[idx].inv_hessian_idepth_idepth * C));
          } else {
            landmark.setIdepthVariance(1e-5_p);
          }
          landmark.setNumberOfInlierResidualsInTheLastOptimization(local_landmarks[idx].number_of_inlier_residuals);
          if (local_landmarks[idx].relative_baseline > landmark.relativeBaseline()) {
            landmark.setRelativeBaseline(static_cast<Precision>(local_landmarks[idx].relative_baseline));
          }
        }
      }
    }
  }
}

template <typename Scalar, energy::motion::Motion Motion, model::Model Model, int PatternSize,
          template <int> typename Grid2D, bool OPTIMIZE_POSES, bool OPTIMIZE_IDEPTHS, bool FIRST_ESTIMATE_JACOBIANS,
          int C>
void PhotometricBundleAdjustment<Scalar, Motion, Model, PatternSize, Grid2D, OPTIMIZE_POSES, OPTIMIZE_IDEPTHS,
                                 FIRST_ESTIMATE_JACOBIANS, C>::updateLocalFrame(const track::ActiveKeyframe<Motion>
                                                                                    &frame) {
  auto local_frame = getLocalFrame(frame.timestamp());
  CHECK(local_frame) << "Cannot update frame, there is no local copy in the solver";
  // add matured landmarks
  bool recently_marginalized = frame.isMarginalized() && !local_frame->is_marginalized;
  local_frame->is_marginalized = frame.isMarginalized();
  local_frame->update(frame, frame.connections());
  size_t sensor_id = frame.sensors()[0];
  if (!recently_marginalized) return;
  // remove all marginalized frames, that have no connections with active frames
  for (auto frame_iter = frames_.begin(); frame_iter != frames_.end();) {
    auto &residuals = (*frame_iter)->residuals.at({sensor_id, sensor_id});
    if (!(*frame_iter)->is_marginalized) {
      ++frame_iter;
      continue;
    }
    bool has_connections = false;
    for (auto &target_frame : frames_) {
      if (!target_frame->is_marginalized && residuals.contains(target_frame->id)) {
        has_connections = true;
      }
    }
    if (!has_connections) {
      frame_iter = frames_.erase(frame_iter);
    } else
      ++frame_iter;
  }
}
template <typename Scalar, energy::motion::Motion Motion, model::Model Model, int PatternSize,
          template <int> typename Grid2D, bool OPTIMIZE_POSES, bool OPTIMIZE_IDEPTHS, bool FIRST_ESTIMATE_JACOBIANS,
          int C>
void PhotometricBundleAdjustment<Scalar, Motion, Model, PatternSize, Grid2D, OPTIMIZE_POSES, OPTIMIZE_IDEPTHS,
                                 FIRST_ESTIMATE_JACOBIANS, C>::firstEstimateJacobians() {
  firstEstimateJacobians_<Scalar, Motion, Model, PatternSize, Grid2D, C>(frames_);
}

template <typename Scalar, energy::motion::Motion Motion, model::Model Model, int PatternSize,
          template <int> typename Grid2D, bool OPTIMIZE_POSES, bool OPTIMIZE_IDEPTHS, bool FIRST_ESTIMATE_JACOBIANS,
          int C>
void PhotometricBundleAdjustment<Scalar, Motion, Model, PatternSize, Grid2D, OPTIMIZE_POSES, OPTIMIZE_IDEPTHS,
                                 FIRST_ESTIMATE_JACOBIANS, C>::relinearizeSystem() {
  auto &last_frame = *frames_.back();
  last_frame.T_w_agent_linearization_point = last_frame.tWorldAgent();
  last_frame.affine_brightness0 = last_frame.affineBrightness();
  last_frame.state_eps.setZero();
}

template <typename Scalar, energy::motion::Motion Motion, model::Model Model, int PatternSize,
          template <int> typename Grid2D, bool OPTIMIZE_POSES, bool OPTIMIZE_IDEPTHS, bool FIRST_ESTIMATE_JACOBIANS,
          int C>
void PhotometricBundleAdjustment<Scalar, Motion, Model, PatternSize, Grid2D, OPTIMIZE_POSES, OPTIMIZE_IDEPTHS,
                                 FIRST_ESTIMATE_JACOBIANS, C>::updatePointStatuses(const size_t
                                                                                       minimum_valid_reprojections_num,
                                                                                   const Scalar sigma_huber_loss) {
  std::vector<Scalar> energies;
  energies.reserve(12000);

  for (auto &reference_frame : frames_) {
    for (const auto &sensor_1 : reference_frame->sensors()) {
      for (const auto &sensor_2 : reference_frame->sensors()) {
        for (size_t landmark_index = 0; landmark_index < reference_frame->active_landmarks[sensor_1].size();
             landmark_index++) {
          auto &landmarks = reference_frame->active_landmarks[sensor_1];
          auto &residuals = reference_frame->residuals[{sensor_1, sensor_2}];

          auto &landmark = landmarks[landmark_index];
          if (landmark.is_marginalized) continue;
          for (auto &target_frame : frames_) {
            if (target_frame->is_marginalized or reference_frame->id == target_frame->id) {
              continue;
            }

            if (not reference_frame->residuals[{sensor_1, sensor_2}].contains(target_frame->id)) {
              continue;
            }

            if (landmark_index >= residuals.at(target_frame->id).size()) continue;
            auto &residual = residuals.at(target_frame->id)[landmark_index];
            if (residual.connection_status == track::PointConnectionStatus::kOk) {
              energies.push_back(residual.energy);
            }
          }
        }
      }
    }
  }

  auto third_quartile = static_cast<size_t>(static_cast<double>(energies.size()) * 0.75);
  std::nth_element(energies.begin(), energies.begin() + static_cast<long>(third_quartile), energies.end());
  Scalar energy_threshold =
      energies.empty() ? 0 : energies.at(third_quartile) + sigma_huber_loss * sigma_huber_loss / 2;

  for (auto &reference_frame : frames_) {
    for (const auto &sensor_1 : reference_frame->sensors()) {
      for (const auto &sensor_2 : reference_frame->sensors()) {
        auto &landmarks = reference_frame->active_landmarks[sensor_1];
        tbb::parallel_for(tbb::blocked_range<size_t>(0, landmarks.size()), [&](auto r) {
          for (auto landmark_index = r.begin(); landmark_index != r.end(); ++landmark_index) {
            size_t valid_reprojections = 0;
            auto &residuals = reference_frame->residuals[{sensor_1, sensor_2}];

            auto &landmark = landmarks[landmark_index];
            if (landmark.is_marginalized) continue;
            landmark.number_of_inlier_residuals = 0;
            for (auto &target_frame : frames_) {
              if (target_frame->is_marginalized or reference_frame->id == target_frame->id) {
                continue;
              }

              if (not reference_frame->residuals[{sensor_1, sensor_2}].contains(target_frame->id)) {
                continue;
              }

              Scalar distance =
                  (reference_frame->tWorldAgent().translation() - target_frame->tWorldAgent().translation()).norm();

              if (landmark_index >= residuals.at(target_frame->id).size()) continue;
              auto &residual = residuals.at(target_frame->id)[landmark_index];
              if (residual.energy > energy_threshold) {
                residual = {track::PointConnectionStatus::kOutlier};
              }
              if (residual.connection_status == track::PointConnectionStatus::kOk) {
                landmark.relative_baseline = std::max(landmark.relative_baseline, landmark.idepth * distance);
                valid_reprojections++;
                landmark.number_of_inlier_residuals++;
              }
            }
            if (valid_reprojections < minimum_valid_reprojections_num) {
              landmark.is_outlier = true;
            }
          }
        });
      }
    }
  }
}

template <typename Scalar, energy::motion::Motion Motion, model::Model Model, int PatternSize,
          template <int> typename Grid2D, bool OPTIMIZE_POSES, bool OPTIMIZE_IDEPTHS, bool FIRST_ESTIMATE_JACOBIANS,
          int C>
PhotometricBundleAdjustment<Scalar, Motion, Model, PatternSize, Grid2D, OPTIMIZE_POSES, OPTIMIZE_IDEPTHS,
                            FIRST_ESTIMATE_JACOBIANS, C>::~PhotometricBundleAdjustment() = default;

#define PBAInstantiation(Motion, Model, PatchSize, Grid, OPTIMIZE_POSES, OPTIMIZE_IDEPTHS, FIRST_ESTIMATE_JACOBIANS, \
                         C)                                                                                          \
  template class PhotometricBundleAdjustment<double, energy::motion::Motion<Precision>, model::Model<Precision>,     \
                                             PatchSize, features::Grid, OPTIMIZE_POSES, OPTIMIZE_IDEPTHS,            \
                                             FIRST_ESTIMATE_JACOBIANS, C>;                                           \
  template class PhotometricBundleAdjustment<float, energy::motion::Motion<Precision>, model::Model<Precision>,      \
                                             PatchSize, features::Grid, OPTIMIZE_POSES, OPTIMIZE_IDEPTHS,            \
                                             FIRST_ESTIMATE_JACOBIANS, C>

PBAInstantiation(SE3, PinholeCamera, Pattern::kSize, PixelMap, true, true, true, 1);
PBAInstantiation(SE3, PinholeCamera, Pattern::kSize, PixelMap, true, true, false, 1);
PBAInstantiation(SE3, PinholeCamera, Pattern::kSize, PixelMap, true, false, false, 1);
PBAInstantiation(SE3, PinholeCamera, Pattern::kSize, PixelMap, false, false, false, 1);
PBAInstantiation(SE3, PinholeCamera, Pattern::kSize, CeresGrid, true, false, false, 1);
PBAInstantiation(SE3, PinholeCamera, 1, PixelMap, true, false, false, 1);
PBAInstantiation(SE3, PinholeCamera, 1, CeresGrid, true, false, false, 1);
PBAInstantiation(SE3, SimpleRadialCamera, Pattern::kSize, PixelMap, true, true, true, 1);
PBAInstantiation(SE3, SimpleRadialCamera, Pattern::kSize, PixelMap, true, true, false, 1);
PBAInstantiation(SE3, SimpleRadialCamera, 1, PixelMap, true, false, false, 1);

PBAInstantiation(SE3, PinholeCamera, 1, PixelMap, true, false, false, 32);
PBAInstantiation(SE3, PinholeCamera, 1, PixelMap, true, false, false, 128);

#undef PBAInstantiation

}  // namespace problem
}  // namespace energy
}  // namespace dsopp
