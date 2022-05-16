#include "tracker/monocular/monocular_tracker.hpp"

#include <functional>
#include <memory>
#include <numbers>

#include "common/image_tools/conversion.hpp"
#include "common/settings.hpp"
#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/camera_model/pinhole/simple_radial.hpp"
#include "energy/motion/se3_motion.hpp"
#include "energy/problems/depth_map.hpp"
#include "energy/problems/photometric_bundle_adjustment/ceres_photometric_bundle_adjustment.hpp"
#include "energy/problems/photometric_bundle_adjustment/eigen_photometric_bundle_adjustment.hpp"
#include "energy/problems/photometric_bundle_adjustment/photometric_bundle_adjustment.hpp"
#include "energy/problems/pose_alignment/eigen_pose_alignment.hpp"
#include "energy/problems/pose_alignment/pose_alignment.hpp"
#include "energy/problems/pose_alignment/precalculated_pose_alignment.hpp"
#include "energy/projector/camera_reproject.hpp"
#include "feature_based_slam/tracker/monocular_tracker.hpp"
#include "feature_based_slam/tracker/tracker.hpp"
#include "features/camera/camera_features.hpp"
#include "features/camera/ceres_grid.hpp"
#include "features/camera/pattern_patch.hpp"
#include "features/camera/pixel_data_frame.hpp"
#include "features/camera/pixel_map.hpp"
#include "features/camera/tracking_feature.hpp"
#include "features/camera/tracking_features_frame.hpp"
#include "marginalization/frame_marginalization_strategy.hpp"
#include "measures/similarity_measure_ssd.hpp"
#include "output_interfaces/image_output_interface.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"

#include "track/active_odometry_track.hpp"
#include "track/active_track.hpp"
#include "track/connections/connections_container.hpp"
#include "track/connections/frame_connection.hpp"
#include "track/frames/active_keyframe.hpp"
#include "track/frames/frame.hpp"
#include "track/landmarks/active_tracking_landmark.hpp"
#include "track/landmarks/immature_tracking_landmark.hpp"
#include "tracker/build_features.hpp"
#include "tracker/create_depth_maps.hpp"
#include "tracker/depth_estimators/depth_estimation.hpp"
#include "tracker/keyframe_strategy/tracker_keyframe_strategy.hpp"
#include "tracker/landmarks_activator/landmarks_activator.hpp"

namespace dsopp {
namespace tracker {
namespace {

template <energy::motion::Motion Motion, energy::model::Model Model>
void pushNewKeyframe(track::ActiveOdometryTrack<Motion> &track, size_t frame_id, time timestamp,
                     const Motion &t_world_agent, const size_t sensor, features::Pyramid &&pyramid,
                     features::CameraFeatures::PyramidOfMasks &&pyramid_of_masks,
                     const features::TrackingFeaturesFrame &features_frame, const Model &model,
                     std::unique_ptr<cv::Mat> &&semantics_data, const cv::Mat &raw_image,
                     const Eigen::Vector2<Precision> &affine_brightness = Eigen::Vector2<Precision>::Zero(),
                     bool save_images_to_track = false) {
  track.pushFrame(frame_id, timestamp, t_world_agent, affine_brightness);

  if (save_images_to_track) track.lastKeyframe().pushImage(sensor, raw_image);
  track.lastKeyframe().pushPyramid(sensor, std::move(pyramid));
  if (semantics_data.get()) {
    track.lastKeyframe().pushSemanticsData(sensor, std::move(semantics_data));
  }

  track.lastKeyframe().pushImmatureLandmarks(sensor, buildFeatures(features_frame, model));

  track.lastKeyframe().pushPyramidOfMasks(sensor, std::move(pyramid_of_masks));
}

template <energy::motion::Motion Motion, energy::model::Model Model, typename DepthEstimator,
          template <int> typename Grid2D, int C>

void estimateDepths(track::ActiveOdometryTrack<Motion> &track, const Motion &t_world_agent,
                    const Eigen::Vector2<Precision> &target_affine_brightness, const size_t &sensor,
                    const Grid2D<C> &pyramid, const sensors::calibration::CameraCalibration &calibration,
                    const sensors::calibration::CameraMask &camera_mask, const Precision sigma_huber_loss) {
  std::vector<std::reference_wrapper<track::landmarks::ImmatureTrackingLandmark>> landmarks;
  for (size_t k = 0; k < track.activeFrames().size(); k++) {
    auto &keyframe = track.getActiveKeyframe(k);
    auto t_keyframe_agent = keyframe.tWorldAgent().inverse() * t_world_agent;
    const auto &reference_affine_brightness = keyframe.affineBrightness();

    landmarks.clear();

    for (size_t i = 0; i < keyframe.immatureLandmarks(sensor).size(); ++i) {
      auto &landmark = keyframe.getImmatureLandmark(sensor, i);
      if (landmark.status() != track::landmarks::ImmatureStatus::kDelete) {
        landmarks.push_back(landmark);
      }
    }

    DepthEstimator::template estimate<typename Motion::Product, Grid2D, Model, 1>(
        pyramid, landmarks, t_keyframe_agent.inverse(), reference_affine_brightness, target_affine_brightness,
        calibration, camera_mask, sigma_huber_loss);
  }
}

template <energy::motion::MotionProduct MotionProduct, energy::model::Model Model>
Precision calculateMeanSquareOpticalFlow(const energy::problem::DepthMap &reference_depth_map,
                                         const MotionProduct &t_t_r, const Model &model) {
  static const int kBorderSize = 4;
  static const Precision kMinIdepth = 1e-6_p;
  Precision square_optical_flow = 0;
  size_t n = 0;
  energy::reprojection::ArrayReprojector<Precision, Model, MotionProduct> reprojector(model, t_t_r);
  long width = reference_depth_map.map.rows(), height = reference_depth_map.map.cols();

  for (int y = kBorderSize; y < height - kBorderSize; y++) {
    for (int x = kBorderSize; x < width - kBorderSize; x++) {
      if (reference_depth_map.map(x, y).weight > 0) {
        Precision idepth = reference_depth_map.map(x, y).idepth / reference_depth_map.map(x, y).weight;
        if (idepth < kMinIdepth) {
          continue;
        }
        Eigen::Vector2<Precision> reprojection(0, 0), coordinates(x, y);
        if (reprojector.reproject(coordinates, idepth, reprojection)) {
          Eigen::Vector3<Precision> coordinates_ray, reprojection_ray;
          model.unproject(coordinates, coordinates_ray);
          model.unproject(reprojection, reprojection_ray);
          square_optical_flow += (coordinates_ray - reprojection_ray).squaredNorm();
          ++n;
        }
      }
    }
  }

  return std::sqrt(square_optical_flow / static_cast<Precision>(n));
}

template <energy::motion::Motion Motion>
std::vector<Motion> initializationPoses(const track::ActiveOdometryTrack<Motion> &track) {
  if (track.frames().size() < 2) return {Motion()};
  const Precision kMinAnglePerturbation = 1._p * std::numbers::pi_v<Precision> / 180._p;
  const Precision kMaxAnglePerturbation = 3. * std::numbers::pi_v<Precision> / 180.;
  const Precision kAnglePerturbationsStep = 0.5_p * std::numbers::pi_v<Precision> / 180._p;

  std::vector<Motion> initializations;
  auto t_w_r = track.getFrame(-1)->tWorldAgent();
  auto t_r_t_prev = track.getFrame(-2)->tWorldAgent().inverse() * track.getFrame(-1)->tWorldAgent();
  // previous motion
  initializations.push_back(t_w_r * t_r_t_prev);
  // double previous motion (frame skipped)
  initializations.push_back(t_w_r * t_r_t_prev * t_r_t_prev);
  // half motion
  initializations.push_back(t_w_r * Motion::Product::exp(t_r_t_prev.log() * 0.5));
  // zero motion
  initializations.push_back(t_w_r);
  // zero motion from keyframe
  initializations.push_back(track.lastKeyframe().tWorldAgent());

  // perturbate previous motion
  for (Precision delta = kMinAnglePerturbation; delta < kMaxAnglePerturbation; delta += kAnglePerturbationsStep) {
    for (Precision delta_rx : {0_p, delta, -delta}) {
      for (Precision delta_ry : {0_p, delta, -delta}) {
        for (Precision delta_rz : {0_p, delta, -delta}) {
          initializations.push_back(
              initializations[0] *
              typename Motion::Product(Sophus::SE3<Precision>(
                  Sophus::SO3<Precision>::exp({delta_rx, delta_ry, delta_rz}), Eigen::Vector3<Precision>(0, 0, 0))));
        }
      }
    }
  }

  return initializations;
}

template <energy::motion::Motion Motion, energy::model::Model Model, template <int> typename Grid2D, int PatternSize,
          int C>
bool estimatePose(track::ActiveOdometryTrack<Motion> &track,
                  energy::problem::PoseAlignment<Motion, Model, PatternSize, Grid2D, C> *solver,
                  const typename track::ActiveKeyframe<Motion>::Pyramids &pyramids, time timestamp,
                  const std::vector<sensors::calibration::CameraMask> &pyramid_of_masks, Motion &t_w_t,
                  typename Motion::Product &t_r_t, Eigen::Vector2<Precision> &affine_brightness, const size_t sensor_id,
                  const sensors::calibration::CameraCalibration &calibration,
                  const std::map<size_t, std::vector<energy::problem::DepthMap>> &reference_frame_depth_map,
                  const size_t number_of_threads, std::vector<Precision> &rmse_last_pose_estimation) {
  const Precision kEnergyRatioThreshold = 2.5_p;
  std::vector<Precision> local_rmse_last_pose_estimation;
  auto initializations = initializationPoses(track);
  bool success = false;

  Motion t_w_t_constant_optimized;
  Eigen::Vector2<Precision> affine_brightness_constant_optimized;

  for (size_t try_number = 0; !success && try_number < initializations.size(); try_number++) {
    int coarse_lvl = static_cast<int>(pyramids.begin()->second.size() - 1);
    success = true;
    t_w_t = initializations[try_number];
    affine_brightness = track.empty() ? Eigen::Vector2<Precision>::Zero() : track.getFrame(-1)->affineBrightness();
    local_rmse_last_pose_estimation = rmse_last_pose_estimation;
    for (int lvl = coarse_lvl; success && lvl >= 0; lvl--) {
      std::map<size_t, const sensors::calibration::CameraMask &> masks;
      masks.insert({sensor_id, pyramid_of_masks.at(static_cast<size_t>(lvl))});
      solver->reset();
      auto model = calibration.cameraModel<Model>(static_cast<size_t>(lvl));
      if (!track.empty()) {
        const auto &last_kf = track.lastKeyframe();
        solver->pushFrame(last_kf.timestamp(), last_kf.tWorldAgent(), last_kf.pyramids(), masks,
                          reference_frame_depth_map, last_kf.affineBrightness(), static_cast<size_t>(lvl), *model,
                          energy::problem::FrameParameterization::kFixed);
      }
      solver->pushFrame(timestamp, t_w_t, pyramids, masks, affine_brightness, static_cast<size_t>(lvl), *model,
                        energy::problem::FrameParameterization::kFree);

      Precision rmse = solver->solve(number_of_threads);
      if (rmse < kEnergyRatioThreshold * local_rmse_last_pose_estimation[static_cast<size_t>(lvl)]) {
        t_w_t = solver->getPose(timestamp);
        affine_brightness = solver->getAffineBrightness(timestamp);
        if (rmse != energy::problem::PoseAlignment<Motion, Model, PatternSize, Grid2D, C>::kZeroCost) {
          local_rmse_last_pose_estimation[static_cast<size_t>(lvl)] = rmse;
        }
      } else {
        success = false;
        VLOG(1) << "Re-tracking try " << try_number << ", level " << lvl
                << " failed. Energy excess = " << rmse / local_rmse_last_pose_estimation[static_cast<size_t>(lvl)];
      }
    }

    if (try_number == 0) {
      t_w_t_constant_optimized = t_w_t;
      affine_brightness_constant_optimized = affine_brightness;
    }
  }

  if (!success) {
    t_w_t = t_w_t_constant_optimized;
    affine_brightness = affine_brightness_constant_optimized;
    // if pose invalid, use maximum energy
    std::transform(rmse_last_pose_estimation.begin(), rmse_last_pose_estimation.end(),
                   rmse_last_pose_estimation.begin(),
                   [&](Precision &element) { return element * kEnergyRatioThreshold; });
    VLOG(1) << "Invalid pose estimating, assume const motion";
  } else {
    rmse_last_pose_estimation = local_rmse_last_pose_estimation;
  }

  t_r_t = track.empty() ? typename Motion::Product() : track.lastKeyframe().tWorldAgent().inverse() * t_w_t;

  return success;
}

template <energy::motion::Motion Motion>
void refinePoses(auto &solver, track::ActiveOdometryTrack<Motion> &track, const size_t number_of_threads) {
  solver.solve(number_of_threads);
  for (size_t i = 0; i < track.activeFrames().size(); i++) {
    solver.updateFrame(*track.activeFrames()[i]);
  }
}
template <energy::motion::Motion Motion>
void updateSolver(auto &solver, track::ActiveOdometryTrack<Motion> &track) {
  for (size_t i = 0; i < track.activeFrames().size(); i++) {
    solver.updateLocalFrame(*track.activeFrames()[i]);
  }
}
template <energy::model::Model Model, energy::motion::Motion Motion>
void addSemanticObservations(track::ActiveKeyframe<Motion> &reference_frame,
                             track::ActiveKeyframe<Motion> &target_frame, const Model &model, const auto &statuses) {
  const energy::reprojection::ArrayReprojector<Precision, Model, typename Motion::Product> reprojector(
      model, target_frame.tWorldAgent().inverse() * reference_frame.tWorldAgent());
  auto *semantics = target_frame.semanticsData(0);
  if (!semantics) return;
  CHECK_GE(reference_frame.activeLandmarks(0).size(), statuses->size());
  for (size_t i = 0; i < statuses->size(); i++) {
    if (statuses->at(i) != track::PointConnectionStatus::kOk) continue;
    auto &landmark = reference_frame.getActiveLandmark(0, i);
    Eigen::Matrix<Precision, 2, Pattern::kSize> pattern_reference;
    Eigen::Matrix<Precision, 2, Pattern::kSize> pattern;
    features::PatternPatch::shiftPattern(landmark.projection(), pattern_reference);
    bool valid = reprojector.reproject(pattern_reference, landmark.idepth(), pattern);
    if (valid) {
      for (int point_idx = 0; point_idx < Pattern::kSize; point_idx++) {
        uchar semantic_class = semantics->template at<uchar>(int(pattern(1, point_idx)), int(pattern(0, point_idx)));
        landmark.addSemanticTypeObservation(static_cast<size_t>(semantic_class));
      }
    }
  }
}
template <energy::model::Model Model, energy::motion::Motion Motion>
void addSemanticObservations(track::ActiveOdometryTrack<Motion> &track, std::vector<size_t> &marginalized_keyframes_ids,
                             const Model &model) {
  for (size_t marginalized_keyframes_id : marginalized_keyframes_ids) {
    auto &reference_frame = *track.keyframes().at(marginalized_keyframes_id);
    for (auto &[target_keyframe_id, connection] : reference_frame.connections()) {
      auto statuses_reference = &connection->referenceReprojectionStatuses(0, 0);
      auto statuses_target = &connection->targetReprojectionStatuses(0, 0);
      if (target_keyframe_id < reference_frame.keyframeId()) {
        std::swap(statuses_reference, statuses_target);
      }
      track::ActiveKeyframe<Motion> &target_frame = *track.keyframes()[target_keyframe_id];
      if (target_frame.isMarginalized()) {
        continue;
      }
      addSemanticObservations<Model, Motion>(reference_frame, target_frame, model, statuses_reference);
      addSemanticObservations<Model, Motion>(target_frame, reference_frame, model, statuses_target);
    }
  }
}

template <energy::motion::Motion Motion>
std::vector<size_t> marginalize(auto &solver, track::ActiveOdometryTrack<Motion> &track,
                                marginalization::FrameMarginalizationStrategy<Motion> &marginalizer) {
  std::vector<size_t> marginalized_keyframes_ids;
  size_t number_of_active_frames_before_marg = track.activeFrames().size();
  marginalizer.marginalize(track);
  size_t number_of_new_marginalized_frames = number_of_active_frames_before_marg - track.activeFrames().size();
  size_t number_of_marginalized_frames = track.marginalizedFrames().size();
  for (size_t i = 0; i < number_of_new_marginalized_frames; ++i) {
    auto &marginalized_frame = *track.marginalizedFrames().at(number_of_marginalized_frames - 1 - i);
    solver.updateLocalFrame(marginalized_frame);
    marginalized_keyframes_ids.push_back(marginalized_frame.keyframeId());
  }
  return marginalized_keyframes_ids;
}

cv::Mat debugCurrentFrame(auto &features) {
  const cv::Mat image = features.frameData();
  const cv::Mat mask = features.pyramidOfMasks()[0].data() == 0;
  cv::Mat red;
  cv::Mat debug;
  cv::cvtColor(image, debug, cv::COLOR_GRAY2BGR);
  cv::cvtColor(mask, red, cv::COLOR_GRAY2BGR);
  red = (red - cv::Scalar(0, 0, 255)) / 2;
  debug = debug - red;
  return debug;
}

cv::Mat debugCurrentKeyframe(auto &features, const energy::problem::DepthMap &depth_map,
                             Precision &visualization_maximum_idepth) {
  cv::Mat debug;
  cv::cvtColor(features.frameData(), debug, cv::COLOR_GRAY2BGR);

  const auto [current_frame_total_idepth, current_frame_features_num] =
      std::accumulate(depth_map.map.reshaped().begin(), depth_map.map.reshaped().end(), std::make_pair(0.0_p, 0.0_p),
                      [](auto sum, const auto point) {
                        if (point.idepth > 0) {
                          return std::make_pair(sum.first + point.idepth / point.weight, sum.second + 1);
                        }
                        return sum;
                      });
  Precision current_frame_average_idepth = current_frame_total_idepth / current_frame_features_num;
  const Precision kSmoothingFactor = 0.9_p;
  if (visualization_maximum_idepth == 0) {
    visualization_maximum_idepth = current_frame_average_idepth * 2;
  }
  // apply exponential filter so colors do not shacking from frame to frame
  visualization_maximum_idepth =
      kSmoothingFactor * visualization_maximum_idepth + (1.0_p - kSmoothingFactor) * (current_frame_average_idepth * 2);

  cv::Mat debug_depth_map(debug.rows, debug.cols, CV_8UC1, cv::Scalar(127));
  cv::Mat mask(debug.rows, debug.cols, CV_8UC1, cv::Scalar(0));
  for (int col = 0; col < depth_map.map.cols(); col++) {
    for (int row = 0; row < depth_map.map.rows(); row++) {
      const auto &point = depth_map.map(row, col);
      if (point.idepth > 0) {
        Precision color =
            std::clamp(255.0_p * (point.idepth / point.weight) / visualization_maximum_idepth, 0.0_p, 255.0_p);
        cv::circle(debug_depth_map, cv::Point(row, col), 3, cv::Scalar(color), cv::FILLED, cv::LINE_8);
        cv::circle(mask, cv::Point(row, col), 3, cv::Scalar(1), cv::FILLED, cv::LINE_8);
      }
    }
  }
  cv::Mat debug_depth_jet = cv::Mat::zeros(debug.rows, debug.cols, CV_8UC3);
  cv::applyColorMap(debug_depth_map, debug_depth_jet, cv::COLORMAP_JET);
  debug_depth_jet.copyTo(debug, mask);
  return debug;
}
}  // namespace

template <energy::motion::Motion Motion, energy::model::Model Model, typename DepthEstimator,
          template <int> typename Grid2D, bool FRAME_EMBEDDER>
requires tracker::DepthEstimator<DepthEstimator, typename Motion::Product>
MonocularTracker<Motion, Model, DepthEstimator, Grid2D, FRAME_EMBEDDER>::MonocularTracker(
    size_t sensor_id, const sensors::calibration::CameraCalibration &calibration, bool use_imu_prior,
    bool save_images_to_track,
    std::unique_ptr<keyframe_strategy::TrackerKeyframeStrategy<Motion>> &&tracker_keyframe_strategy,
    std::unique_ptr<marginalization::FrameMarginalizationStrategy<Motion>> &&frame_marginalization_strategy,
    std::unique_ptr<energy::problem::PhotometricBundleAdjustment<Precision, Motion, Model, Pattern::kSize, Grid2D, true,
                                                                 true, true, C>> &&photometric_bundle_adjustment,
    std::unique_ptr<energy::problem::PoseAlignment<Motion, Model, kFrontendPatternSize, Grid2D,
                                                   kFrontendChannelsNumber>> &&pose_aligner,
    std::unique_ptr<LandmarksActivator<Motion, Model, Grid2D, 1, true>> &&landmarks_activator)
    : sensor_id_(sensor_id),
      calibration_(calibration),
      use_imu_prior_(use_imu_prior),
      save_images_to_track_(save_images_to_track),
      tracker_keyframe_strategy_(std::move(tracker_keyframe_strategy)),
      marginalizer_(std::move(frame_marginalization_strategy)),
      landmarks_activator_(std::move(landmarks_activator)),
      rmse_last_pose_estimation_(sensors::calibration::CameraCalibration::kNumberOfPyramidLevels, 1e5),
      photometric_bundle_adjustment_(std::move(photometric_bundle_adjustment)),
      pose_aligner_(std::move(pose_aligner)) {}

template <energy::motion::Motion Motion, energy::model::Model Model, typename DepthEstimator,
          template <int> typename Grid2D, bool FRAME_EMBEDDER>
requires tracker::DepthEstimator<DepthEstimator, typename Motion::Product> void
MonocularTracker<Motion, Model, DepthEstimator, Grid2D, FRAME_EMBEDDER>::initialize(
    const feature_based_slam::tracker::Tracker &tracker,
    const std::vector<std::unique_ptr<sensors::SynchronizedFrame>> &frames, track::ActiveTrack<Motion> &track,
    const size_t number_of_threads) {
  auto storaged_frame = frames.begin();
  for (size_t i = 0; i < tracker.evaluatedFramesSize(); i++) {
    while ((*storaged_frame)->timestamp() != tracker.timestamp(i)) {
      storaged_frame++;
    }

    pose_aligner_->pushKnownPose((*storaged_frame)->timestamp(), tracker.tWorldAgent(i));
    pose_aligner_->pushKnownPose(tracker.timestamp(0), tracker.tWorldAgent(0));

    tick(track, **storaged_frame, number_of_threads, true, i == tracker.evaluatedFramesSize() - 1);
  }
  this->is_initialized_ = true;
}

template <energy::motion::Motion Motion, energy::model::Model Model, typename DepthEstimator,
          template <int> typename Grid2D, bool FRAME_EMBEDDER>
requires tracker::DepthEstimator<DepthEstimator, typename Motion::Product> void
MonocularTracker<Motion, Model, DepthEstimator, Grid2D, FRAME_EMBEDDER>::tick(
    track::ActiveTrack<Motion> &track, const sensors::SynchronizedFrame &frame_data, const size_t number_of_threads,
    bool initialization, bool force_keyframe) {
  auto &camera_features = frame_data.cameraFeatures();
  if (!camera_features.contains(sensor_id_)) {
    return;
  }

  auto &features = *camera_features.at(sensor_id_);

  auto &odometry_track = track.odometryTrack();

  if (!this->is_initialized_ && !initialization) {
    LOG(ERROR) << "Monocular tracker uninitialized!";
  } else {
    typename track::ActiveKeyframe<Motion>::Pyramids pyramids;
    auto pixel_data = features.movePixelData();
    pyramids[sensor_id_] = features::movePyramidAndDelete(pixel_data);
    const auto &pyramid_of_masks = features.pyramidOfMasks();
    Motion t_w_t;
    typename Motion::Product t_r_t;
    Eigen::Vector2<Precision> affine_brightness = Eigen::Vector2<Precision>::Zero();

    estimatePose<Motion, Model, Grid2D, kFrontendPatternSize, kFrontendChannelsNumber>(
        odometry_track, pose_aligner_.get(), pyramids, features.timestamp(), pyramid_of_masks, t_w_t, t_r_t,
        affine_brightness, sensor_id_, calibration_, reference_frame_depth_map_, number_of_threads,
        rmse_last_pose_estimation_);

    auto model = calibration_.cameraModel<Model>();

    if (odometry_track.activeFrames().empty()) {
      auto &tracking_features_frame = features.tracking();
      pushNewKeyframe<Motion, Model>(odometry_track, features.id(), features.timestamp(), t_w_t, sensor_id_,
                                     std::move(pyramids[sensor_id_]), features.movePyramidOfMasks(),
                                     tracking_features_frame, *model, features.moveSemanticsData(), features.image(),
                                     affine_brightness, save_images_to_track_);
      photometric_bundle_adjustment_->pushFrame(odometry_track.lastKeyframe(), 0, *model,
                                                energy::problem::FrameParameterization::kFixed);
      updateSolver(*photometric_bundle_adjustment_, odometry_track);
      reference_frame_depth_map_ = createReferenceDepthMaps<Motion, Model>(odometry_track.activeFrames(), calibration_);
      return;
    }

    estimateDepths<Motion, Model, DepthEstimator, Grid2D, 1>(odometry_track, t_w_t, affine_brightness, sensor_id_,
                                                             (pyramids[sensor_id_])[0], calibration_,
                                                             pyramid_of_masks[0], kHuberLossSigma);

    auto t_t_r = t_r_t.inverse();
    Precision mean_square_optical_flow = calculateMeanSquareOpticalFlow<typename Motion::Product, Model>(
        reference_frame_depth_map_.at(sensor_id_)[0], t_t_r, *model);
    t_t_r.setRotationMatrix(Eigen::Matrix3<Precision>::Identity());
    Precision mean_square_optical_flow_without_rotation =
        calculateMeanSquareOpticalFlow<typename Motion::Product, Model>(reference_frame_depth_map_.at(sensor_id_)[0],
                                                                        t_t_r, *model);
    auto t_w_r = t_w_t * t_t_r;
    auto frame = track::SLAMInternalTrackingFrame(features.id(), features.timestamp(), t_w_r, t_r_t, affine_brightness,
                                                  mean_square_optical_flow, mean_square_optical_flow_without_rotation,
                                                  rmse_last_pose_estimation_[0]);

    if (this->camera_output_interface_.current_frame) {
      this->camera_output_interface_.current_frame->pushImage(debugCurrentFrame(features));
    }

    if (tracker_keyframe_strategy_->needNewKeyframe(odometry_track, frame) || force_keyframe) {
      auto &tracking_features_frame = features.tracking();
      pushNewKeyframe<Motion, Model>(odometry_track, features.id(), features.timestamp(), t_w_t, sensor_id_,
                                     std::move(pyramids[sensor_id_]), features.movePyramidOfMasks(),
                                     tracking_features_frame, *model, features.moveSemanticsData(), features.image(),
                                     affine_brightness, save_images_to_track_);
      landmarks_activator_->activate(odometry_track);

      photometric_bundle_adjustment_->pushFrame(odometry_track.lastKeyframe(), 0, *model);

      updateSolver(*photometric_bundle_adjustment_, odometry_track);

      refinePoses(*photometric_bundle_adjustment_, odometry_track, number_of_threads);
      affine_brightness = odometry_track.lastKeyframe().affineBrightness();

      odometry_track.unloadMarginalizedResources();
      auto marginalized_keyframes_ids = marginalize(*photometric_bundle_adjustment_, odometry_track, *marginalizer_);
      addSemanticObservations<Model>(odometry_track, marginalized_keyframes_ids, *model);
      updateSolver(*photometric_bundle_adjustment_, odometry_track);

      reference_frame_depth_map_ = createReferenceDepthMaps<Motion, Model>(odometry_track.activeFrames(), calibration_);

      if (this->camera_output_interface_.current_keyframe) {
        this->camera_output_interface_.current_keyframe->pushImage(
            debugCurrentKeyframe(features, reference_frame_depth_map_.at(sensor_id_)[0], maximum_visualized_idepth_));
      }

    } else {
      odometry_track.lastKeyframe().attachTrackingFrame(
          features.id(), features.timestamp(), t_r_t, affine_brightness, mean_square_optical_flow,
          mean_square_optical_flow_without_rotation, rmse_last_pose_estimation_[0]);
      // TODO add option to save images to attached frames
      // odometry_track.lastKeyframe().lastAttachedFrame().pushImage(sensor_id_,
      //                                                            features.image());
    }
  }
}

template <energy::motion::Motion Motion, energy::model::Model Model, typename DepthEstimator,
          template <int> typename Grid2D, bool FRAME_EMBEDDER>
requires tracker::DepthEstimator<DepthEstimator, typename Motion::Product> bool
MonocularTracker<Motion, Model, DepthEstimator, Grid2D, FRAME_EMBEDDER>::frameEmbedder() const {
  return FRAME_EMBEDDER;
}

template <energy::motion::Motion Motion, energy::model::Model Model, typename DepthEstimator,
          template <int> typename Grid2D, bool FRAME_EMBEDDER>
requires tracker::DepthEstimator<DepthEstimator, typename Motion::Product>
    MonocularTracker<Motion, Model, DepthEstimator, Grid2D, FRAME_EMBEDDER>::~MonocularTracker() = default;

template class MonocularTracker<energy::motion::SE3<Precision>, energy::model::PinholeCamera<Precision>,
                                tracker::DepthEstimation, features::PixelMap, false>;

template class MonocularTracker<energy::motion::SE3<Precision>, energy::model::SimpleRadialCamera<Precision>,
                                tracker::DepthEstimation, features::PixelMap, false>;

template class MonocularTracker<energy::motion::SE3<Precision>, energy::model::PinholeCamera<Precision>,
                                tracker::DepthEstimation, features::PixelMap, true>;

template class MonocularTracker<energy::motion::SE3<Precision>, energy::model::SimpleRadialCamera<Precision>,
                                tracker::DepthEstimation, features::PixelMap, true>;

}  // namespace tracker
}  // namespace dsopp
