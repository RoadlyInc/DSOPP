
#include <ctime>
#include <iomanip>

#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include "common/file_tools/camera_frame_times.hpp"
#include "common/settings.hpp"
#include "common/time/time.hpp"
#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"
#include "energy/projector/camera_reproject.hpp"
#include "features/camera/camera_features.hpp"
#include "features/camera/frame_embedding_extractor.hpp"
#include "features/camera/pattern_patch.hpp"
#include "features/camera/pixel_data_frame.hpp"
#include "features/camera/pixel_map.hpp"
#include "features/camera/sobel_tracking_features_extractor.hpp"
#include "features/camera/tracking_feature.hpp"
#include "sensors/camera/camera.hpp"
#include "sensors/camera_calibration/fabric.hpp"

#include "sensor/synchronized_frame.hpp"
#include "sensors/camera_providers/image_video_provider.hpp"
#include "test/tools/depth_gt.hpp"
#include "test/tools/solver_test_data.hpp"
#include "test/tools/tum_gt.hpp"
#include "track/active_odometry_track.hpp"
#include "track/frames/active_keyframe.hpp"
#include "track/frames/slam_internal_tracking_frame.hpp"
#include "track/landmarks/immature_tracking_landmark.hpp"
#include "tracker/build_features.hpp"
#include "tracker/depth_estimators/depth_estimation.hpp"
#include "tracker/keyframe_strategy/frequency_keyframe_strategy.hpp"
#include "tracker/landmarks_activator/landmarks_activator.hpp"

#define PRINT_TIME_INFO COMPILE_HIDDEN_CODE
#define PRINT_TINY_IDEPTH_VALUES COMPILE_HIDDEN_CODE
#define PRINT_HUGE_IDEPTH_VALUES COMPILE_HIDDEN_CODE

using namespace dsopp;
using time_point = std::chrono::system_clock::time_point;
using SE3 = dsopp::energy::motion::SE3<Precision>;

std::vector<Eigen::Vector3<Precision>> createPointsFromImmatureLandmarks(
    const std::vector<track::landmarks::ImmatureTrackingLandmark>& landmarks);

template <energy::model::Model Model>
void statistics(const std::vector<std::vector<Precision>>& depth_gt,
                const std::vector<Eigen::Vector3<Precision>>& landmarks, size_t vicinity_radius, const SE3& t_t_r,
                const sensors::calibration::CameraCalibration& camera_calibration, Precision& deviation_pixel,
                Precision& deviation_depth, Precision& percent_of_outliers);
Precision findClosestDepthInVicinity(const Eigen::Vector<Precision, 2>& point_coordinates,
                                     const std::vector<std::vector<Precision>>& depth_gt, size_t vicinity_radius,
                                     Precision depth);
int main() {
  size_t current_frame_id = 0;
  using Calibration = sensors::calibration::CameraCalibration;
  using Model = energy::model::PinholeCamera<Precision>;
  static constexpr energy::model::ModelType ModelType = energy::model::ModelType::kPinholeCamera;
  auto calibration = Calibration(Eigen::Vector2<Precision>(1280, 720),
                                 Eigen::Vector4<Precision>(448.15_p, 448.15_p, 640, 360), ModelType);
  auto photometric_calibration =
      sensors::calibration::photometric_calibration::create(TEST_DATA_DIR "track30seconds/pcalib.txt");
  auto vignetting = sensors::calibration::vignetting::create(TEST_DATA_DIR "track30seconds/vignetting.png");
  auto provider = std::make_unique<sensors::providers::ImageVideoProvider>(
      TEST_DATA_DIR "track30seconds/images.mkv", TEST_DATA_DIR "track30seconds/times.csv", 10);
  auto model = calibration.cameraModel<Model>();

  Eigen::Vector2<int> im_size = calibration.image_size().cast<int>();
  auto camera_mask = sensors::calibration::CameraMask(im_size.y(), im_size.x());

  auto settings = std::make_unique<sensors::calibration::CameraSettings>(
      std::move(calibration), std::move(photometric_calibration), std::move(vignetting), std::move(camera_mask));
  sensors::Camera camera("camera_1", 0, *settings, std::move(provider),
                         std::make_unique<dsopp::features::SobelTrackingFeaturesExtractor>());
  Precision outliers_pixels(0);

  track::ActiveOdometryTrack<SE3> track;

  Precision frequency = 3;
  auto kf_strategy = std::make_unique<tracker::keyframe_strategy::FrequencyKeyframeStrategy<SE3>>(frequency);

  auto camera_poses_gt = test_tools::TumGt(TEST_DATA_DIR "track30seconds/gt.tum");
  auto camera_depths_gt =
      test_tools::DepthGt(TEST_DATA_DIR "track30seconds/CameraDepth/", TEST_DATA_DIR "track30seconds/times.csv");
  Precision depth_deviation, pixel_deviation;
  size_t sensor = 0;

  // first pose
  SE3 gt_world_cam;
  {
    sensors::SynchronizedFrame synchronized_frame(camera.nextFrameId(), camera.nextFrameTime());
    camera.processNextDataFrame(synchronized_frame);
    auto& features = *synchronized_frame.cameraFeatures().at(sensor);

    gt_world_cam = camera_poses_gt.getPose(features.timestamp());
    track.pushFrame(current_frame_id++, features.timestamp(), gt_world_cam);
    {
      auto pyramids = features.movePixelData();
      track.lastKeyframe().pushPyramid(sensor, features::movePyramidAndDelete(pyramids));
    }
    track.lastKeyframe().pushImmatureLandmarks(sensor, tracker::buildFeatures(features.tracking(), *model));
  }
  Precision average_number_of_landmarks = 0, average_depth_deviation = 0, average_pixel_deviation = 0,
            average_outliers_pixels = 0, number_of_ready_landmarks = 0;
  for (int i = 0; i < 5000; i++) {
    sensors::SynchronizedFrame synchronized_frame(camera.nextFrameId(), camera.nextFrameTime());
    if (!camera.processNextDataFrame(synchronized_frame)) {
      break;
    }
    auto& features = *synchronized_frame.cameraFeatures().at(sensor);

    gt_world_cam = camera_poses_gt.getPose(features.timestamp());

    track::SLAMInternalTrackingFrame slam_internal_frame(current_frame_id++, features.timestamp(), SE3(), SE3(), 1,
                                                         Eigen::Vector2<Precision>::Zero(), 0, 0);

    if (kf_strategy->needNewKeyframe(track, slam_internal_frame)) {
      // add new kf
      track.pushFrame(current_frame_id++, features.timestamp(), gt_world_cam);
      {
        auto pyramids = features.movePixelData();
        track.lastKeyframe().pushPyramid(sensor, features::movePyramidAndDelete(pyramids));
      }
      track.lastKeyframe().pushImmatureLandmarks(sensor, tracker::buildFeatures(features.tracking(), *model));

      // calculate metric
      if (track.frames().size() > 1) {
        size_t vicinity_radius = 1;  // radius in pixels
        auto track_frames_number = track.frames().size();
#if PRINT_TIME_INFO
        std::cout << "first frame timestamps: " << track.frames()[track_frames_number - 2]->timestamp() << "   ";
        std::cout << "second frame timestamps: " << track.frames()[track_frames_number - 1]->timestamp() << "   ";
#endif
        auto camera_depth_gt = camera_depths_gt.getFrame(track.frames()[track_frames_number - 2]->timestamp());
        auto points =
            createPointsFromImmatureLandmarks(track.frames()[track_frames_number - 2]->immatureLandmarks(sensor));
        SE3 t_t_r = track.frames()[track_frames_number - 2]->tWorldAgent().inverse() *
                    track.frames()[track_frames_number - 1]->tWorldAgent();
        statistics<Model>(*camera_depth_gt, points, vicinity_radius, t_t_r, camera.calibration(), pixel_deviation,
                          depth_deviation, outliers_pixels);

        std::cout << "frame timestamp is: " << track.frames()[track_frames_number - 2]->timestamp() << ";\n";
        std::cout << "total number of ready to activation landmarks: " << points.size() << ";\n";
        average_number_of_landmarks += static_cast<Precision>(points.size());
        std::cout << "depth deviation is: " << depth_deviation << std::endl;
        average_depth_deviation += depth_deviation;
        std::cout << "pixel deviation is: " << pixel_deviation << std::endl;
        average_pixel_deviation += pixel_deviation;
        std::cout << "percentage of outliers: " << outliers_pixels * 100.0 << std::endl;
        average_outliers_pixels += outliers_pixels * 100.0_p;
        std::cout << "====================================================================" << std::endl;
        number_of_ready_landmarks++;
      }
    } else {
      auto t_t_r = gt_world_cam.inverse() * track.frames().back()->tWorldAgent();
      track.lastKeyframe().attachTrackingFrame(current_frame_id++, features.timestamp(), t_t_r.inverse());
      const features::PixelMap<1>& target_grid = features.pixelData().getLevel(0);

      const size_t landmarksSize = track.lastKeyframe().immatureLandmarks(sensor).size();
      std::vector<std::reference_wrapper<track::landmarks::ImmatureTrackingLandmark>> landmarks;

      for (size_t landmark_i = 0; landmark_i < landmarksSize; ++landmark_i) {
        auto& landmark = track.lastKeyframe().getImmatureLandmark(sensor, landmark_i);
        landmarks.push_back(landmark);
      }
      tracker::DepthEstimation::estimate<SE3, features::PixelMap, Model, 1>(
          target_grid, landmarks, t_t_r, 1, Eigen::Vector2<Precision>::Zero(), 1, Eigen::Vector2<Precision>::Zero(),
          camera.calibration(), camera.pyramidOfMasks()[0], 9);
    }
  }
  std::cout << "====================================================================" << std::endl;
  std::cout << "====================================================================" << std::endl;
  std::cout << "Average number of ready to activation landmarks: "
            << average_number_of_landmarks / number_of_ready_landmarks << ";\n";
  std::cout << "Average depth deviation is: " << average_depth_deviation / number_of_ready_landmarks << std::endl;
  std::cout << "Average pixel deviation is: " << average_pixel_deviation / number_of_ready_landmarks << std::endl;
  std::cout << "Average percentage of outliers: " << average_outliers_pixels / number_of_ready_landmarks << std::endl;
  std::cout << "====================================================================" << std::endl;

  return 0;
}

template <energy::model::Model Model>
void statistics(const std::vector<std::vector<Precision>>& depth_gt,
                const std::vector<Eigen::Vector3<Precision>>& points, size_t vicinity_radius, const SE3& t_t_r,
                const sensors::calibration::CameraCalibration& camera_calibration, Precision& deviation_pixel,
                Precision& deviation_depth, Precision& percent_of_outliers) {
  const Precision kOutlierThreshold = 3;
  size_t number_of_landmarks = points.size();
  Precision idepth_estimated, depth_gt_min;
  std::vector<Precision> squared_differences_pixel;
  std::vector<Precision> squared_differences_depth;
  Eigen::Vector2<Precision> landmark_coordinates, reprojected_gt_coordinates, reprojected_estimated_coordinates,
      pixel_error;
  bool reprojection_success_estimated, reprojection_success_gt;
  auto camera_model = camera_calibration.cameraModel<Model>();
  size_t number_of_outliers = 0;

  for (size_t idx = 0; idx < number_of_landmarks; ++idx) {
    camera_model->project(points[idx], landmark_coordinates);
    idepth_estimated = 1._p / points[idx][2];
    if (std::abs(idepth_estimated) > 1000) {
#if PRINT_HUGE_IDEPTH_VALUES
      std::cout << "DEBUG: idepth is huge: " << idepth_estimated << std::endl;
#endif
      continue;
    }
    if (std::abs(idepth_estimated) < 0.001) {
#if PRINT_TINY_IDEPTH_VALUES
      std::cout << "DEBUG: idepth is tiny: " << idepth_estimated << std::endl;
#endif
      continue;
    }
    depth_gt_min =
        findClosestDepthInVicinity(landmark_coordinates, depth_gt, vicinity_radius, 1.0_p / idepth_estimated);

    energy::reprojection::ArrayReprojector<Precision, Model, energy::motion::SE3<Precision>> reprojector(*camera_model,
                                                                                                         t_t_r);

    reprojection_success_estimated =
        reprojector.reproject(landmark_coordinates, idepth_estimated, reprojected_estimated_coordinates);
    reprojection_success_gt =
        reprojector.reproject(landmark_coordinates, 1.0_p / depth_gt_min, reprojected_gt_coordinates);

    Precision depth_difference = 1.0_p / idepth_estimated - depth_gt_min;
    pixel_error = (reprojected_estimated_coordinates - reprojected_gt_coordinates).cwiseAbs();
    if (reprojection_success_estimated and reprojection_success_gt) {
      if (pixel_error.norm() < kOutlierThreshold) {
        squared_differences_pixel.push_back(pixel_error.squaredNorm());
        squared_differences_depth.push_back(depth_difference * depth_difference);
      } else {
        number_of_outliers += 1;
      }
    }
  }

  Precision sum = std::accumulate(squared_differences_pixel.begin(), squared_differences_pixel.end(), 0.0_p);
  deviation_pixel = std::sqrt(sum / static_cast<Precision>(squared_differences_pixel.size() - 1));
  sum = std::accumulate(squared_differences_depth.begin(), squared_differences_depth.end(), 0.0_p);
  deviation_depth = std::sqrt(sum / static_cast<Precision>(squared_differences_depth.size() - 1));
  percent_of_outliers = static_cast<Precision>(number_of_outliers) / static_cast<Precision>(number_of_landmarks);
}

Precision findClosestDepthInVicinity(const Eigen::Vector<Precision, 2>& point_coordinates,
                                     const std::vector<std::vector<Precision>>& depth_gt, size_t vicinity_radius,
                                     Precision depth) {
  int width = static_cast<int>(depth_gt.size());
  int height = static_cast<int>(depth_gt[0].size());
  int half_window = static_cast<int>(vicinity_radius);
  Precision closest_depth = 1e5;
  int point_coordinate_x = static_cast<int>(point_coordinates(0));
  int point_coordinate_y = static_cast<int>(point_coordinates(1));

  for (int idx_x = -half_window; idx_x <= half_window; ++idx_x) {
    for (int idx_y = -half_window; idx_y <= half_window; ++idx_y) {
      if (point_coordinate_x + idx_x >= 0 and point_coordinate_x + idx_x < width and point_coordinate_y + idx_y >= 0 and
          point_coordinate_y + idx_y < height) {
        Precision current_depth = depth_gt[static_cast<size_t>(point_coordinates(0) + static_cast<Precision>(idx_x))]
                                          [static_cast<size_t>(point_coordinates(1) + static_cast<Precision>(idx_y))];
        if (std::abs(closest_depth - depth) > std::abs(current_depth - depth)) {
          closest_depth = current_depth;
        }
      }
    }
  }
  return closest_depth;
}

std::vector<Eigen::Vector3<Precision>> createPointsFromImmatureLandmarks(
    const std::vector<track::landmarks::ImmatureTrackingLandmark>& landmarks) {
  std::vector<Eigen::Vector3<Precision>> points;
  for (const auto& landmark : landmarks) {
    if (landmark.isTraced() && landmark.readyForActivation()) {
      points.emplace_back(landmark.direction() / landmark.idepth());
    }
  }
  return points;
}
