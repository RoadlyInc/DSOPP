#include "tracker/landmarks_activator/landmarks_activator.hpp"

#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/camera_model/pinhole/simple_radial.hpp"
#include "energy/levenberg_marquardt_algorithm/levenberg_marquardt_algorithm.hpp"

#include "energy/problems/photometric_bundle_adjustment/ceres_photometric_bundle_adjustment.hpp"
#include "energy/projector/camera_reproject.hpp"
#include "features/camera/ceres_grid.hpp"
#include "features/camera/pixel_map.hpp"
#include "measures/similarity_measure_ssd.hpp"

#include "track/active_odometry_track.hpp"
#include "track/frames/active_keyframe.hpp"
#include "track/landmarks/active_tracking_landmark.hpp"
#include "track/landmarks/immature_tracking_landmark.hpp"

namespace dsopp {
namespace tracker {

namespace {
/**
 * function to choose new sparsity coefficient (minimum distance between projections of immature and active points of
 * the last frame). The P-regulator used to choose new coefficient (new value of the coefficient proportional to the
 * difference between desired number of active points and current number of active points).
 * @param level level to get
 * @return pyramid level
 */
void recalculateMinDistanceToNeighbor(size_t number_of_active_points, size_t number_of_desired_points,
                                      Precision &min_distance_to_neighbor) {
  const Precision kPRegulatorCoefficient = 0.001_p;
  const Precision kMinDistance = 0;
  const Precision kMaxDistance = 10;
  min_distance_to_neighbor +=
      (static_cast<Precision>(number_of_active_points) - static_cast<Precision>(number_of_desired_points)) *
      kPRegulatorCoefficient;
  min_distance_to_neighbor = std::clamp(min_distance_to_neighbor, kMinDistance, kMaxDistance);
}

bool haveNoNeighbors(const Eigen::Vector2<Precision> &projection,
                     const std::vector<Eigen::Vector2<Precision>> &reprojected, Precision distance) {
  for (const auto &landmark : reprojected) {
    if ((landmark - projection).norm() < distance) {
      return false;
    }
  }
  return true;
}

template <energy::motion::Motion Motion, typename Model>
size_t reprojectActivePoints(const track::ActiveOdometryTrack<Motion> &track, const Model &model,
                             std::vector<Eigen::Vector2<Precision>> &reprojected,
                             const size_t level_to_maintain_sparsity) {
  size_t number_of_active_points = 0;
  for (const auto &frame : track.activeFrames()) {
    if (frame->id() == track.activeFrames().back()->id()) {
      continue;
    }
    auto t_t_r = track.activeFrames().back()->tWorldAgent().inverse() * frame->tWorldAgent();
    energy::reprojection::ArrayReprojector<Precision, Model, typename Motion::Product> reprojector(model, t_t_r);
    for (const auto &sensor : frame->sensors()) {
      const auto &target_mask = track.activeFrames().back()->getMask(sensor, level_to_maintain_sparsity);
      for (const auto &landmark : frame->activeLandmarks(sensor)) {
        if (landmark.isOutlier() or landmark.isMarginalized()) continue;

        Eigen::Vector2<Precision> point_2d;
        number_of_active_points++;

        if (!reprojector.reproject(landmark.projection() / (1 << level_to_maintain_sparsity), landmark.idepth(),
                                   point_2d)) {
          continue;
        }

        if (!target_mask.valid(point_2d)) {
          continue;
        }

        reprojected.push_back(point_2d);
      }
    }
  }
  return number_of_active_points;
}

template <energy::motion::Motion Motion>
typename track::ActiveKeyframe<Motion>::ImmatureLandmarkActivationStatus activationStatus(
    const track::landmarks::ImmatureTrackingLandmark &landmark, const auto &reprojector,
    std::vector<Eigen::Vector2<Precision>> &reprojected, const Precision min_distance_to_neighbor,
    const sensors::calibration::CameraMask &target_mask, const size_t level_to_maintain_sparsity) {
  using ImmaturePointActivationAction = typename track::ActiveKeyframe<Motion>::ImmatureLandmarkActivationStatus;
  if (landmark.status() == track::landmarks::ImmatureStatus::kDelete) {
    return ImmaturePointActivationAction::kDelete;
  }
  if (!landmark.isTraced() || landmark.status() == track::landmarks::ImmatureStatus::kOutlier) {
    return ImmaturePointActivationAction::kDelete;
  }
  if (!landmark.readyForActivation()) {
    if (landmark.status() == track::landmarks::ImmatureStatus::kOutOfBoundary) {
      return ImmaturePointActivationAction::kDelete;
    } else {
      return ImmaturePointActivationAction::kSkip;
    }
  }

  Eigen::Vector2<Precision> point_2d;
  if (reprojector.reproject(landmark.projection() / (1 << level_to_maintain_sparsity), landmark.idepth(), point_2d)) {
    if (!target_mask.valid(point_2d)) {
      return ImmaturePointActivationAction::kDelete;
      ;
    }
    if (haveNoNeighbors(point_2d, reprojected, min_distance_to_neighbor)) {
      reprojected.push_back(point_2d);
      return ImmaturePointActivationAction::kActivate;
    } else {
      return ImmaturePointActivationAction::kSkip;
    }
  } else {
    return ImmaturePointActivationAction::kDelete;
  }
}

template <energy::motion::Motion Motion, typename Model, template <int> typename Grid2D, int C>
class LandmarkActivationProblem {
  constexpr static Precision kMaxEnergyForInliers = Pattern::kSize * 12 * 12;

 public:
  LandmarkActivationProblem(const Eigen::Matrix<Precision, Pattern::kSize, 1> &reference_patch,
                            size_t reference_frame_id, const Eigen::Vector2<Precision> &reference_affine_brightness,
                            size_t sensor, const std::deque<track::ActiveKeyframe<Motion> *> &frames,
                            const Model &model, const Precision sigma_huber_loss,
                            const Eigen::Matrix<Precision, 2, Pattern::kSize> &reference_pattern, const Motion &t_w_r,
                            Precision &idepth)
      : reference_patch_(reference_patch),
        reference_frame_id_(reference_frame_id),
        reference_affine_brightness_(reference_affine_brightness),
        sensor_(sensor),
        frames_(frames),
        model_(model),
        sigma_huber_loss_(sigma_huber_loss),
        reference_pattern_(reference_pattern),
        t_w_r_(t_w_r),
        idepth_(idepth),
        old_idepth_(idepth) {}

  std::pair<Precision, int> calculateEnergy() {
    Precision energy = 0;
    int number_of_valid_residuals = 0;
    if (stop_) {
      idepth_ = -1;
      return {energy, number_of_valid_residuals};
    }

    for (auto &target_frame : frames_) {
      if (reference_frame_id_ == target_frame->id()) {
        continue;
      }
      const auto &target_mask = target_frame->getMask(sensor_, 0);
      auto &target_affine_brightness = target_frame->affineBrightness();
      Precision brightness_change_scale = std::exp(target_affine_brightness[0] - reference_affine_brightness_[0]);

      auto t_t_r = target_frame->tWorldAgent().inverse() * t_w_r_;
      energy::reprojection::ArrayReprojector<Precision, Model, typename Motion::Product> reprojector(model_, t_t_r);
      Eigen::Matrix<Precision, 2, Pattern::kSize> target_pattern;

      bool success = reprojector.reprojectPattern(reference_pattern_, idepth_, target_pattern);
      success = success && target_mask.valid(target_pattern);

      if (success) {
        Eigen::Vector<Precision, Pattern::kSize> target_patch;
        Eigen::Vector<Precision, Pattern::kSize> residuals;

        target_frame->getLevel(sensor_, 0).Evaluate(target_pattern, target_patch);
        measure::SimilarityMeasureSSD::residuals(
            target_patch - Eigen::Vector<Precision, Pattern::kSize>::Constant(target_affine_brightness[1]),
            brightness_change_scale * (reference_patch_ - Eigen::Vector<Precision, Pattern::kSize>::Constant(
                                                              reference_affine_brightness_[1])),
            residuals);
        Precision huber_weight = residuals.norm() > sigma_huber_loss_ ? sigma_huber_loss_ / residuals.norm() : 1;

        if (residuals.squaredNorm() < kMaxEnergyForInliers) {
          energy += huber_weight * residuals.squaredNorm();
          number_of_valid_residuals++;
        } else {
          energy += kMaxEnergyForInliers;
        }
      }
    }

    if (number_of_valid_residuals == 0) {
      idepth_ = -1;
      stop_ = true;
    }

    return {energy, number_of_valid_residuals};
  }

  void linearize() {
    hessian_ = 0;
    b_ = 0;

    for (auto &target_frame : frames_) {
      if (reference_frame_id_ == target_frame->id()) {
        continue;
      }
      const auto &target_mask = target_frame->getMask(sensor_, 0);
      auto &target_affine_brightness = target_frame->affineBrightness();
      Precision brightness_change_scale = std::exp(target_affine_brightness[0] - reference_affine_brightness_[0]);

      auto t_t_r = target_frame->tWorldAgent().inverse() * t_w_r_;
      energy::reprojection::ArrayReprojector<Precision, Model, typename Motion::Product> reprojector(model_, t_t_r);

      Eigen::Matrix<Precision, 2, Pattern::kSize> target_pattern;
      Eigen::Vector<Precision, Pattern::kSize> d_u_idepth;
      Eigen::Vector<Precision, Pattern::kSize> d_v_idepth;
      Eigen::Matrix<Precision, Pattern::kSize, Motion::Product::DoF> d_v_tReferenceTarget;
      Eigen::Matrix<Precision, Pattern::kSize, Motion::Product::DoF> d_u_tReferenceTarget;
      bool success = reprojector.reprojectPattern(reference_pattern_, idepth_, target_pattern, d_u_idepth, d_v_idepth,
                                                  d_u_tReferenceTarget, d_v_tReferenceTarget);
      success = success && target_mask.valid(target_pattern);

      if (success) {
        Eigen::Matrix<Precision, Pattern::kSize, 1> target_patch;
        Eigen::Vector<Precision, Pattern::kSize> d_intensity_u;
        Eigen::Vector<Precision, Pattern::kSize> d_intensity_v;

        target_frame->getLevel(sensor_, 0).Evaluate(target_pattern, target_patch, d_intensity_u, d_intensity_v);

        Eigen::Vector<Precision, Pattern::kSize> residuals;
        measure::SimilarityMeasureSSD::residuals(
            target_patch - Eigen::Vector<Precision, Pattern::kSize>::Constant(target_affine_brightness[1]),
            brightness_change_scale * (reference_patch_ - Eigen::Vector<Precision, Pattern::kSize>::Constant(
                                                              reference_affine_brightness_[1])),
            residuals);
        Precision huber_weight = residuals.norm() > sigma_huber_loss_ ? sigma_huber_loss_ / residuals.norm() : 1;

        Eigen::Vector<Precision, Pattern::kSize> d_idepth =
            (d_intensity_u.array() * d_u_idepth.array() + d_intensity_v.array() * d_v_idepth.array());

        hessian_ += huber_weight * d_idepth.transpose() * d_idepth;
        b_ += huber_weight * d_idepth.transpose() * residuals;
      }
    }
    if (hessian_ == 0) stop_ = true;
  }

  void calculateStep(const Precision levenberg_marquardt_regularizer) {
    step_ = b_ / (hessian_ + hessian_ * levenberg_marquardt_regularizer);
    old_idepth_ = idepth_;
    idepth_ -= step_;
  }

  std::pair<Precision, Precision> acceptStep() { return {idepth_ * idepth_, step_ * step_}; }

  void rejectStep() { idepth_ = old_idepth_; }

  bool stop() { return stop_; }

 private:
  const Eigen::Matrix<Precision, Pattern::kSize, 1> &reference_patch_;
  size_t reference_frame_id_;
  const Eigen::Vector2<Precision> &reference_affine_brightness_;
  size_t sensor_;
  const std::deque<track::ActiveKeyframe<Motion> *> &frames_;
  const Model &model_;
  const Precision sigma_huber_loss_;
  const Eigen::Matrix<Precision, 2, Pattern::kSize> &reference_pattern_;
  const Motion &t_w_r_;
  Precision &idepth_;
  Precision old_idepth_;
  Precision hessian_ = 0;
  Precision b_ = 0;
  Precision step_ = 0;
  bool stop_ = false;
};

template <energy::motion::Motion Motion, typename Model, template <int> typename Grid2D, int C>
typename track::ActiveKeyframe<Motion>::ImmatureLandmarkActivationStatus optimizeImmatureLandmark(
    track::landmarks::ImmatureTrackingLandmark &landmark, size_t reference_frame_id,
    const Eigen::Vector2<Precision> &reference_affine_brightness, size_t sensor,
    const std::deque<track::ActiveKeyframe<Motion> *> &frames, const Model &model, const int minimum_inliers,
    const Motion &t_w_r, const Precision sigma_huber_loss) {
  using ImmaturePointActivationAction = typename track::ActiveKeyframe<Motion>::ImmatureLandmarkActivationStatus;

  energy::levenberg_marquardt_algorithm::Options options;
  options.initial_levenberg_marquardt_regularizer = 1._p / 10;
  options.function_tolerance = 0;
  options.parameter_tolerance = 1e-8_p;
  options.max_num_iterations = 3;
  options.levenberg_marquardt_regularizer_decrease_on_accept = 2.;
  options.levenberg_marquardt_regularizer_increase_on_reject = 5.;

  Eigen::Matrix<Precision, 2, Pattern::kSize> reference_pattern;
  features::PatternPatch::shiftPattern(landmark.projection(), reference_pattern);
  Precision idepth = landmark.idepth();
  LandmarkActivationProblem<Motion, Model, Grid2D, C> problem(landmark.patch(), reference_frame_id,
                                                              reference_affine_brightness, sensor, frames, model,
                                                              sigma_huber_loss, reference_pattern, t_w_r, idepth);
  auto result = energy::levenberg_marquardt_algorithm::solve(problem, options);

  if (result.number_of_valid_residuals < minimum_inliers || idepth < 0) {
    return ImmaturePointActivationAction::kDelete;
  } else {
    landmark.setIdepthMin(idepth);
    landmark.setIdepthMax(idepth);
    return ImmaturePointActivationAction::kActivate;
  }
}

template <energy::motion::Motion Motion, typename Model, template <int> typename Grid2D, int C>
void optimizeImmatureLandmarks(
    const track::ActiveOdometryTrack<Motion> &track, const Model &model,
    std::map<size_t,
             std::map<size_t, std::vector<typename track::ActiveKeyframe<Motion>::ImmatureLandmarkActivationStatus>>>
        &statuses,
    const int minimum_inliers, const Precision sigma_huber_loss) {
  using ImmaturePointActivationAction = typename track::ActiveKeyframe<Motion>::ImmatureLandmarkActivationStatus;

  for (size_t reference_frame_idx = 0; reference_frame_idx + 1 < track.activeFrames().size(); ++reference_frame_idx) {
    auto &reference_frame = track.getActiveKeyframe(reference_frame_idx);
    for (auto sensor : reference_frame.sensors()) {
      for (size_t landmark_idx = 0; landmark_idx < reference_frame.immatureLandmarks(sensor).size(); ++landmark_idx) {
        auto &status = statuses.at(reference_frame.id()).at(sensor).at(landmark_idx);
        if (status != ImmaturePointActivationAction::kActivate) {
          continue;
        }
        status = optimizeImmatureLandmark<Motion, Model, Grid2D, C>(
            reference_frame.getImmatureLandmark(sensor, landmark_idx), reference_frame.id(),
            reference_frame.affineBrightness(), sensor, track.activeFrames(), model,
            std::min(minimum_inliers, static_cast<int>(track.activeFrames().size()) - 1), reference_frame.tWorldAgent(),
            sigma_huber_loss);
      }
    }
  }
}
}  // namespace

template <energy::motion::Motion Motion, energy::model::Model Model, template <int> typename Grid2D, int C, bool REFINE>
LandmarksActivator<Motion, Model, Grid2D, C, REFINE>::LandmarksActivator(
    const sensors::calibration::CameraCalibration &calibration, const Precision sigma_huber_loss,
    const size_t number_of_desired_points)
    : calibration_(calibration),
      sigma_huber_loss_(sigma_huber_loss),
      number_of_desired_points_(number_of_desired_points) {}

template <energy::motion::Motion Motion, energy::model::Model Model, template <int> typename Grid2D, int C, bool REFINE>
void LandmarksActivator<Motion, Model, Grid2D, C, REFINE>::activate(track::ActiveOdometryTrack<Motion> &track) {
  const size_t kLevelToMaintainSparsity = 1;
  const int kMinimumInliers = 1;

  using ImmaturePointActivationAction = typename track::ActiveKeyframe<Motion>::ImmatureLandmarkActivationStatus;
  auto model = calibration_.cameraModel<Model>(kLevelToMaintainSparsity);
  std::map<size_t, std::map<size_t, std::vector<ImmaturePointActivationAction>>> statuses;
  std::vector<Eigen::Vector2<Precision>> reprojected;

  size_t number_of_active_points = reprojectActivePoints(track, *model, reprojected, kLevelToMaintainSparsity);

  recalculateMinDistanceToNeighbor(number_of_active_points, number_of_desired_points_, min_distance_to_neighbor_);

  for (const auto &frame : track.activeFrames()) {
    if (frame->id() == track.activeFrames().back()->id()) {
      continue;
    }
    std::map<size_t, std::vector<ImmaturePointActivationAction>> frame_statuses;
    for (const auto &sensor : frame->sensors()) {
      const auto &target_mask = track.activeFrames().back()->getMask(sensor, kLevelToMaintainSparsity);
      std::vector<ImmaturePointActivationAction> frame_sensor_statuses;
      auto t_t_r = track.activeFrames().back()->tWorldAgent().inverse() * frame->tWorldAgent();
      energy::reprojection::ArrayReprojector<Precision, Model, typename Motion::Product> reprojector(*model, t_t_r);
      for (const auto &landmark : frame->immatureLandmarks(sensor)) {
        auto status = activationStatus<Motion>(landmark, reprojector, reprojected, min_distance_to_neighbor_,
                                               target_mask, kLevelToMaintainSparsity);
        frame_sensor_statuses.push_back(status);
      }
      frame_statuses[sensor] = std::move(frame_sensor_statuses);
    }
    statuses[frame->id()] = std::move(frame_statuses);
  }

  if constexpr (REFINE) {
    auto optimization_model = calibration_.cameraModel<Model>();
    optimizeImmatureLandmarks<Motion, Model, Grid2D, C>(track, *optimization_model, statuses, kMinimumInliers,
                                                        sigma_huber_loss_);
  }

  track.applyImmatureLandmarkActivationStatuses(statuses);
}

#define LandmarksActivatorInstantiation(Model, Motion, Optimize)                                        \
  template class LandmarksActivator<energy::motion::Motion<Precision>, energy::model::Model<Precision>, \
                                    features::PixelMap, 1, Optimize>;                                   \
  template class LandmarksActivator<energy::motion::Motion<Precision>, energy::model::Model<Precision>, \
                                    features::CeresGrid, 1, Optimize>

LandmarksActivatorInstantiation(PinholeCamera, SE3, true);
LandmarksActivatorInstantiation(PinholeCamera, SE3, false);
LandmarksActivatorInstantiation(SimpleRadialCamera, SE3, true);
LandmarksActivatorInstantiation(SimpleRadialCamera, SE3, false);
#undef LandmarksActivatorInstantiation

}  // namespace tracker
}  // namespace dsopp
