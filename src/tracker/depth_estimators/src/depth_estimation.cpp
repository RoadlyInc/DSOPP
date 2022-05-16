#include "tracker/depth_estimators/depth_estimation.hpp"

#include <functional>

#include <tbb/parallel_for.h>

#include "common/settings.hpp"
#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/camera_model/pinhole/simple_radial.hpp"
#include "energy/epipolar_geometry/epipolar_line_builder.hpp"
#include "energy/levenberg_marquardt_algorithm/levenberg_marquardt_algorithm.hpp"
#include "energy/motion/se3_motion.hpp"
#include "energy/projector/camera_reproject.hpp"
#include "features/camera/pattern_patch.hpp"
#include "features/camera/pixel_map.hpp"
#include "features/camera/tracking_feature.hpp"
#include "features/camera/tracking_features_frame.hpp"
#include "measures/similarity_measure_ssd.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"

#include "track/landmarks/immature_tracking_landmark.hpp"

namespace dsopp {
namespace tracker {
namespace {
Precision calculateError(const track::landmarks::ImmatureTrackingLandmark &landmark,
                         const Eigen::Vector2<Precision> &epiline_vector) {
  // TODO: Rewrite it in a more understandable way
  Eigen::Vector2<Precision> epiline_vector_orth = Eigen::Vector2<Precision>(epiline_vector(1), -epiline_vector(0));
  Precision a = std::pow(epiline_vector.dot(landmark.gradient()), 2_p);
  Precision b = std::pow(epiline_vector_orth.dot(landmark.gradient()), 2_p);
  return 0.2_p + 0.2_p * (a + b) / a;
}

template <energy::motion::MotionProduct MotionProduct, typename Grid2D, energy::model::Model Model>
void findBest(const std::vector<energy::epipolar_geometry::EpipolarLine::EpipolarLinePoint> &epiline_points,
              std::vector<Precision> &energies, Precision &best_energy, size_t &optimum,
              const Eigen::Vector<Precision, Pattern::kSize> &reference_pattern,
              const Eigen::Vector2<Precision> &coords, const Grid2D &target_frame, const Model &camera_model,
              const MotionProduct &t_target_reference, const sensors::calibration::CameraMask &camera_mask,
              const Eigen::Vector2<Precision> &reference_affine_brightness,
              const Eigen::Vector2<Precision> &target_affine_brightness, const size_t distance) {
  const energy::reprojection::ArrayReprojector<Precision, Model, MotionProduct> reprojector(camera_model,
                                                                                            t_target_reference);
  Eigen::Matrix<Precision, 2, Pattern::kSize> reference_patch;
  features::PatternPatch::shiftPattern(coords, reference_patch);

  Eigen::Vector<Precision, Pattern::kSize> target_pattern;

  const Eigen::Vector<Precision, Pattern::kSize> reference_pattern_precalc =
      ceres::exp(target_affine_brightness[0] - reference_affine_brightness[0]) *
      (reference_pattern - Eigen::Vector<Precision, Pattern::kSize>::Constant(reference_affine_brightness[1]));

  for (size_t idx = 0; idx < distance; idx++) {
    const auto &point = epiline_points[idx];
    Eigen::Matrix<Precision, 2, Pattern::kSize> target_patch;

    bool success = reprojector.template reprojectPattern(reference_patch, point.reference_idepth, target_patch);

    success = success && camera_mask.valid<false>(point.projection);

    if (success) {
      target_frame.Evaluate(target_patch, target_pattern);
      Precision energy = measure::SimilarityMeasureSSD::calculate(
          target_pattern - Eigen::Vector<Precision, Pattern::kSize>::Constant(target_affine_brightness[1]),
          reference_pattern_precalc);
      energies[idx] = energy;
      if (energy < best_energy) {
        best_energy = energy;
        optimum = idx;
      }
    }
  }
}

template <energy::motion::MotionProduct MotionProduct, template <int> typename Grid2D, energy::model::Model Model,
          int C>
class DepthEstimationProblem {
 public:
  DepthEstimationProblem(const Grid2D<C> &target_frame,
                         const Eigen::Matrix<Precision, Pattern::kSize, 1> &reference_patch,
                         const Eigen::Vector2<Precision> &reference_affine_brightness,
                         const Eigen::Vector2<Precision> &target_affine_brightness, const Model &model,
                         Eigen::Matrix<Precision, 2, Pattern::kSize> &target_pattern,
                         const Eigen::Vector2<Precision> &tangent, const Precision sigma_huber_loss)
      : target_frame_(target_frame),
        reference_patch_(reference_patch),
        reference_affine_brightness_(reference_affine_brightness),
        target_affine_brightness_(target_affine_brightness),
        model_(model),
        target_pattern_(target_pattern),
        old_target_pattern_(target_pattern),
        tangent_(tangent),
        sigma_huber_loss_(sigma_huber_loss),
        reference_pattern_precalc_(
            ceres::exp(target_affine_brightness_[0] - reference_affine_brightness_[0]) *
            (reference_patch_ - Eigen::Vector<Precision, Pattern::kSize>::Constant(reference_affine_brightness_[1]))) {}

  std::pair<Precision, int> calculateEnergy() {
    Eigen::Vector<Precision, Pattern::kSize> target_patch;
    Eigen::Vector<Precision, Pattern::kSize> residuals;

    target_frame_.Evaluate(target_pattern_, target_patch);
    measure::SimilarityMeasureSSD::residuals(
        target_patch - Eigen::Vector<Precision, Pattern::kSize>::Constant(target_affine_brightness_[1]),
        reference_pattern_precalc_, residuals);

    const Precision energy = residuals.cwiseMin(sigma_huber_loss_).cwiseMax(-sigma_huber_loss_).dot(residuals);
    return {energy, 1};
  }

  void linearize() {
    Eigen::Vector<Precision, Pattern::kSize> target_patch;
    Eigen::Vector<Precision, Pattern::kSize> residuals;
    Eigen::Vector<Precision, Pattern::kSize> d_intensity_u;
    Eigen::Vector<Precision, Pattern::kSize> d_intensity_v;

    hessian_ = 0;
    b_ = 0;

    target_frame_.Evaluate(target_pattern_, target_patch, d_intensity_u, d_intensity_v);
    measure::SimilarityMeasureSSD::residuals(
        target_patch - Eigen::Vector<Precision, Pattern::kSize>::Constant(target_affine_brightness_[1]),
        reference_pattern_precalc_, residuals);

    const Eigen::Vector<Precision, Pattern::kSize> huber_weights =
        sigma_huber_loss_ * residuals.cwiseAbs().cwiseMax(sigma_huber_loss_).array().cwiseInverse();

    const Eigen::Vector<Precision, Pattern::kSize> d_intensities_epiline =
        tangent_(0) * d_intensity_u + tangent_(1) * d_intensity_v;

    hessian_ = huber_weights.dot(d_intensities_epiline.cwiseAbs2());
    b_ = huber_weights.dot((residuals.array() * d_intensities_epiline.array()).matrix());
  }

  void calculateStep(const Precision levenberg_marquardt_regularizer) {
    const Precision kMaxStep = 0.3_p;
    step_ = b_ / (hessian_ + hessian_ * levenberg_marquardt_regularizer);
    step_ = std::clamp(step_, -kMaxStep, kMaxStep);

    old_target_pattern_ = target_pattern_;
    target_pattern_.colwise() -= step_ * tangent_;

    if (!model_.insideCameraROI(target_pattern_)) {
      target_pattern_ = old_target_pattern_;
      stop_ = true;
    }
  }

  std::pair<Precision, Precision> acceptStep() const { return {0, step_ * step_}; }

  void rejectStep() { target_pattern_ = old_target_pattern_; }

  bool stop() const { return stop_; }

 private:
  const Grid2D<C> &target_frame_;
  const Eigen::Matrix<Precision, Pattern::kSize, 1> &reference_patch_;
  const Eigen::Vector2<Precision> &reference_affine_brightness_;
  const Eigen::Vector2<Precision> &target_affine_brightness_;
  const Model &model_;
  Eigen::Matrix<Precision, 2, Pattern::kSize> &target_pattern_;
  Eigen::Matrix<Precision, 2, Pattern::kSize> old_target_pattern_;
  const Eigen::Vector2<Precision> &tangent_;
  const Precision sigma_huber_loss_;
  Precision hessian_ = 0;
  Precision b_ = 0;
  Precision step_ = 0;
  bool stop_ = false;

  const Eigen::Vector<Precision, Pattern::kSize> reference_pattern_precalc_;
};

template <energy::motion::MotionProduct MotionProduct, template <int> typename Grid2D, energy::model::Model Model,
          int C>
std::optional<Eigen::Vector2<Precision>> refine(
    const Grid2D<C> &target_frame, track::landmarks::ImmatureTrackingLandmark &reference_landmark,
    const MotionProduct &t_t_r, const Eigen::Vector2<Precision> &reference_affine_brightness,
    const Eigen::Vector2<Precision> &target_affine_brightness, const Model &camera_model,
    const Precision sigma_huber_loss, const Eigen::Vector2<Precision> &tangent,
    const energy::epipolar_geometry::EpipolarLine::EpipolarLinePoint &epipolar_point, Precision &best_energy) {
  energy::levenberg_marquardt_algorithm::Options options;
  options.initial_levenberg_marquardt_regularizer = 2.;
  options.function_tolerance = 0;
  options.parameter_tolerance = 1e-1_p;
  options.max_num_iterations = 3;
  options.levenberg_marquardt_regularizer_decrease_on_accept = 2.;
  options.levenberg_marquardt_regularizer_increase_on_reject = 2.;

  Eigen::Matrix<Precision, 2, Pattern::kSize> target_pattern;
  Eigen::Matrix<Precision, 2, Pattern::kSize> reference_pattern;

  features::PatternPatch::shiftPattern(reference_landmark.projection(), reference_pattern);
  energy::reprojection::ArrayReprojector<Precision, Model, MotionProduct> reprojector(camera_model, t_t_r);

  bool success = reprojector.reprojectPattern(reference_pattern, epipolar_point.reference_idepth, target_pattern);
  if (!success) {
    return {};
  }

  DepthEstimationProblem<MotionProduct, Grid2D, Model, C> problem(
      target_frame, reference_landmark.patch(), reference_affine_brightness, target_affine_brightness, camera_model,
      target_pattern, tangent, sigma_huber_loss);

  auto result = energy::levenberg_marquardt_algorithm::solve(problem, options);
  best_energy = result.energy;

  return target_pattern.col(Pattern::kCenter);
}

template <energy::motion::MotionProduct MotionProduct, template <int> typename Grid2D, energy::model::Model Model,
          int C>
void estimateLandmark(const Grid2D<C> &target_frame, track::landmarks::ImmatureTrackingLandmark &reference_landmark,
                      const MotionProduct &t_t_r, const Eigen::Vector2<Precision> &reference_affine_brightness,
                      const Eigen::Vector2<Precision> &target_affine_brightness,
                      const sensors::calibration::CameraCalibration &calibration,
                      const sensors::calibration::CameraMask &camera_mask, const Precision sigma_huber_loss) {
  const Precision kMinEpilineSize = 2;
  const Precision kMinDepthScale = 0.75;
  const Precision kMaxDepthScale = 1.5;
  const Precision kMaxError = 10;
  const Precision kErrorStep = 10;
  const size_t kUniquenessRadius = 2;
  const size_t kMinEpilineSizeForUniqueness = 10;
  const Precision kMaxEnergyPerPixel = 12 * 12;
  const Precision kMaxEnergyForInliers = Pattern::kSize * kMaxEnergyPerPixel;
  const Precision kEps = 1e-10_p;

  auto camera_model = calibration.cameraModel<Model>();
  const Precision kMaxPixSearch = camera_model->image_size().sum() * 0.027_p;

  if (reference_landmark.status() == track::landmarks::ImmatureStatus::kOutOfBoundary ||
      reference_landmark.status() == track::landmarks::ImmatureStatus::kDelete ||
      reference_landmark.status() == track::landmarks::ImmatureStatus::kOutlier) {
    return;
  }

  const Eigen::Vector2<Precision> &coords = reference_landmark.projection();
  auto epiline_builder = energy::epipolar_geometry::EpipolarLineBuilder<Model, MotionProduct>(*camera_model, t_t_r);
  auto epiline = epiline_builder.buildSegment(coords, reference_landmark.idepthMin(), reference_landmark.idepthMax());
  const auto &epiline_points = epiline.points;
  if (epiline.empty()) {
    reference_landmark.setSearchPixelInterval(0);
    reference_landmark.setStatus(track::landmarks::ImmatureStatus::kOutOfBoundary);
    return;
  }

  Precision search_distance = epiline.length();
  if (search_distance < kMinEpilineSize) {
    reference_landmark.setSearchPixelInterval(search_distance);
    reference_landmark.setStatus(track::landmarks::ImmatureStatus::kSkipped);
    return;
  }

  Precision depth_scale =
      camera_model->getDepthScale(reference_landmark.direction(), epiline_points.front().reference_idepth, t_t_r);
  if ((reference_landmark.idepthMin() >= 0 && (depth_scale < kMinDepthScale || depth_scale > kMaxDepthScale))) {
    reference_landmark.setSearchPixelInterval(0);
    reference_landmark.setStatus(track::landmarks::ImmatureStatus::kOutOfBoundary);
    return;
  }

  size_t optimum = 0;
  Precision best_energy = 1e6;
  std::vector<Precision> energies(epiline_points.size(), std::numeric_limits<Precision>::max());

  size_t distance = epiline_points.size();
  if (!reference_landmark.isTraced()) {
    distance = std::min(
        distance, static_cast<size_t>(kMaxPixSearch / search_distance * static_cast<Precision>(epiline_points.size())));
  }
  findBest<MotionProduct, Grid2D<C>, Model>(epiline_points, energies, best_energy, optimum, reference_landmark.patch(),
                                            coords, target_frame, *camera_model, t_t_r, camera_mask,
                                            reference_affine_brightness, target_affine_brightness, distance);

  Precision second_best_energy = std::numeric_limits<Precision>::max();
  for (size_t idx = 0; idx < epiline_points.size(); idx++) {
    if ((idx + kUniquenessRadius < optimum or idx > optimum + kUniquenessRadius) and energies[idx] < second_best_energy)
      second_best_energy = energies[idx];
  }
  reference_landmark.setUniqueness(second_best_energy / best_energy, search_distance > kMinEpilineSizeForUniqueness);

  // Approxiamte tangent at point optimum
  const Eigen::Vector2<Precision> epiline_vector = epiline.tangent(optimum);

  const Eigen::Vector2<Precision> tangent = epiline_vector.stableNormalized();
  auto refined_point = refine<MotionProduct, Grid2D, Model, C>(
      target_frame, reference_landmark, t_t_r, reference_affine_brightness, target_affine_brightness, *camera_model,
      sigma_huber_loss, tangent, epiline_points[optimum], best_energy);

  if (!refined_point) {
    reference_landmark.setSearchPixelInterval(0);
    reference_landmark.setStatus(track::landmarks::ImmatureStatus::kOutOfBoundary);
    return;
  }

  const Eigen::Vector2<Precision> subpixel_optimum = refined_point.value();
  const Eigen::Vector2<Precision> subpixel_shift = subpixel_optimum - epiline_points[optimum].projection;
  Precision shift = subpixel_shift.norm();
  if ((subpixel_shift.dot(epiline_vector)) < 0) {
    shift *= -1;
  }

  if (best_energy > kMaxEnergyForInliers) {
    reference_landmark.setSearchPixelInterval(0);
    reference_landmark.setStatus(track::landmarks::ImmatureStatus::kOutlier);
    return;
  }

  Precision error = calculateError(reference_landmark, epiline_vector);
  if (error > search_distance / 2 && reference_landmark.isTraced()) {
    reference_landmark.setSearchPixelInterval(search_distance);
    reference_landmark.setStatus(track::landmarks::ImmatureStatus::kIllConditioned);
    return;
  }

  error = std::min(error, kMaxError);
  Precision idepth_min = -1, idepth_max = -1;

  auto triangulator = epiline_builder.getTriangulator(coords);
  Precision error_step = error / kErrorStep;
  while ((!camera_model->validIdepth(idepth_min) || !camera_model->validIdepth(idepth_max)) && error > -kEps) {
    auto point_error_right = epiline.shift(optimum, -error + shift);
    auto point_error_left = epiline.shift(optimum, error + shift);
    idepth_min = triangulator.getInverseDepth(point_error_right.projection);
    idepth_max = triangulator.getInverseDepth(point_error_left.projection);
    error -= error_step;
  }

  if (!camera_model->validIdepth(idepth_min) || !camera_model->validIdepth(idepth_max)) {
    reference_landmark.setSearchPixelInterval(0);
    reference_landmark.setStatus(track::landmarks::ImmatureStatus::kOutOfBoundary);
    return;
  }

  if (idepth_min > idepth_max) {
    std::swap(idepth_min, idepth_max);
  }

  reference_landmark.setIdepthMin(idepth_min);
  reference_landmark.setIdepthMax(idepth_max);
  reference_landmark.setSearchPixelInterval(2 * error_step * kErrorStep);
  reference_landmark.setStatus(track::landmarks::ImmatureStatus::kGood);
}

}  // namespace

template <energy::motion::MotionProduct MotionProduct, template <int> typename Grid2D, energy::model::Model Model,
          int C>
void DepthEstimation::estimate(
    const Grid2D<C> &target_frame,
    std::vector<std::reference_wrapper<track::landmarks::ImmatureTrackingLandmark>> &reference_landmarks,
    const MotionProduct &t_t_r, const Eigen::Vector2<Precision> &reference_affine_brightness,
    const Eigen::Vector2<Precision> &target_affine_brightness,
    const sensors::calibration::CameraCalibration &calibration, const sensors::calibration::CameraMask &camera_mask,
    const Precision sigma_huber_loss) {
  const size_t N = reference_landmarks.size();

  tbb::parallel_for(tbb::blocked_range<size_t>(0, N), [&](auto r) {
    for (auto i = r.begin(); i != r.end(); ++i) {
      auto &landmark = reference_landmarks[i];
      estimateLandmark<MotionProduct, Grid2D, Model, C>(target_frame, landmark, t_t_r, reference_affine_brightness,
                                                        target_affine_brightness, calibration, camera_mask,
                                                        sigma_huber_loss);
    }
  });
}

#define EstimateInstantiation(Model, Motion)                                                                           \
  template void DepthEstimation::estimate<energy::motion::Motion<Precision>, features::PixelMap,                       \
                                          energy::model::Model<Precision>, 1>(                                         \
      const features::PixelMap<1> &,                                                                                   \
      std::vector<std::reference_wrapper<track::landmarks::ImmatureTrackingLandmark>> &,                               \
      const energy::motion::Motion<Precision> &, const Eigen::Vector2<Precision> &, const Eigen::Vector2<Precision> &, \
      const sensors::calibration::CameraCalibration &, const sensors::calibration::CameraMask &, Precision)

EstimateInstantiation(PinholeCamera, SE3);
EstimateInstantiation(SimpleRadialCamera, SE3);
#undef EstimateInstantiation

}  // namespace tracker
}  // namespace dsopp
