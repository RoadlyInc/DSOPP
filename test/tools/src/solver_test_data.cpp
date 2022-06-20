#include "test/tools/solver_test_data.hpp"

#include <functional>

#include "common/settings.hpp"
#include "energy/motion/se3_motion.hpp"
#include "features/camera/camera_features.hpp"
#include "features/camera/ceres_grid.hpp"
#include "features/camera/pixel_map.hpp"
#include "features/camera/sobel_tracking_features_extractor.hpp"
#include "features/camera/tracking_feature.hpp"
#include "features/camera/tracking_features_frame.hpp"
#include "sensor/synchronized_frame.hpp"
#include "sensors/camera_calibration/fabric.hpp"
#include "sensors/camera_calibration/mask/camera_mask.hpp"
#include "sensors/camera_providers/image_video_provider.hpp"
#include "test/tools/depth_gt.hpp"
#include "test/tools/tum_gt.hpp"
#include "track/connections/connections_container.hpp"
#include "track/connections/frame_connection.hpp"
#include "track/landmarks/active_tracking_landmark.hpp"
#include "track/landmarks/immature_tracking_landmark.hpp"
#include "tracker/build_features.hpp"
#include "tracker/landmarks_activator/landmarks_activator.hpp"

namespace dsopp {
namespace test_tools {

template <energy::motion::Motion Motion, template <int> typename Grid2D, bool GN_NET_EMBEDDER>
SolverTestData<Motion, Grid2D, GN_NET_EMBEDDER>::SolverTestData(std::vector<size_t> keyframe_ids, bool add_noise,
                                                                int max_points_per_frame, bool add_connections)
    : gt(TEST_DATA_DIR "track30seconds/gt.tum") {
  const size_t kStartFrame = 10;

  srand(0);
  std::sort(keyframe_ids.begin(), keyframe_ids.end());
  auto calibration = Calibration(Eigen::Vector2<Precision>(1280, 720),
                                 Eigen::Vector4<Precision>(448.155164329_p, 448.155164329_p, 640, 360), ModelType);
  auto photometric_calibration =
      sensors::calibration::photometric_calibration::create(TEST_DATA_DIR "track30seconds/pcalib.txt");
  auto vignetting = sensors::calibration::vignetting::create(TEST_DATA_DIR "track30seconds/vignetting.png");
  auto provider = std::make_unique<sensors::providers::ImageVideoProvider>(
      TEST_DATA_DIR "track30seconds/images.mkv", TEST_DATA_DIR "track30seconds/times.csv", kStartFrame);
  Eigen::Vector2<int> im_size = calibration.image_size().cast<int>();
  auto camera_mask = sensors::calibration::CameraMask(im_size.y(), im_size.x());
  model = calibration.cameraModel<Model>();
  settings = std::make_unique<sensors::calibration::CameraSettings>(
      std::move(calibration), std::move(photometric_calibration), std::move(vignetting), std::move(camera_mask));
  camera = std::make_unique<sensors::Camera>("camera_1", sensor, *settings, std::move(provider),
                                             std::make_unique<dsopp::features::SobelTrackingFeaturesExtractor>(2000));

  auto gt_depth_data =
      test_tools::DepthGt(TEST_DATA_DIR "track30seconds/CameraDepth", TEST_DATA_DIR "track30seconds/times.csv");
  Motion t_world_frame;
  if (keyframe_ids.empty()) return;

  std::vector<std::unique_ptr<sensors::SynchronizedFrame>> frames(keyframe_ids.back() + 1);
  auto& odometry_track = track.odometryTrack();
  for (size_t current_frame = 0; current_frame < keyframe_ids.front(); ++current_frame) {
    frames[current_frame] =
        std::make_unique<sensors::SynchronizedFrame>(camera->nextFrameId(), camera->nextFrameTime());
    camera->processNextDataFrame(*frames[current_frame]);
  }
  auto landmarks_activator =
      std::make_unique<tracker::LandmarksActivator<Motion, Model, Grid2D, 1>>(camera->calibration());
  for (size_t current_frame = keyframe_ids.front(); current_frame <= keyframe_ids.back(); ++current_frame) {
    frames[current_frame] =
        std::make_unique<sensors::SynchronizedFrame>(camera->nextFrameId(), camera->nextFrameTime());
    camera->processNextDataFrame(*frames[current_frame]);

    if (current_frame == keyframe_ids.front()) {
      t_world_frame = gt.getPose(frames[current_frame]->timestamp()).inverse();
    }
    gt_cam.emplace(
        std::make_pair(current_frame, t_world_frame * Motion(gt.getPose(frames[current_frame]->timestamp()))));
    if (std::find(keyframe_ids.begin(), keyframe_ids.end(), current_frame) == keyframe_ids.end()) {
      continue;
    }
    auto features = frames[current_frame]->cameraFeatures().at(sensor).get();
    auto t_cam = t_world_frame * Motion(gt.getPose(features->timestamp()));
    if (add_noise && current_frame != keyframe_ids.front()) {
      const Precision kPoseDeviation = 2e-2_p;
      Eigen::Vector<Precision, Motion::DoF> noise_log =
          Eigen::Vector<Precision, Motion::DoF>::Random() * kPoseDeviation;
      Motion noise = Motion::exp(noise_log);
      t_cam = t_cam * noise;
    }
    auto gt_depth_frame = gt_depth_data.getFrame(features->timestamp());
    gt_depth_maps.emplace(std::make_pair(current_frame, gt_depth_frame));
    odometry_track.pushFrame(features->id(), features->timestamp(), t_cam);
    {
      auto pixel_data_frame = features->movePixelData();
      odometry_track.lastKeyframe().pushPyramid(sensor, features::movePyramidAndDelete(pixel_data_frame));
      odometry_track.lastKeyframe().pushPyramidOfMasks(sensor, features->movePyramidOfMasks());
    }
    odometry_track.lastKeyframe().pushImmatureLandmarks(sensor, tracker::buildFeatures(features->tracking(), *model));

    const size_t landmarksSize = odometry_track.lastKeyframe().immatureLandmarks(sensor).size();
    Precision landmark_pick_probability =
        static_cast<Precision>(max_points_per_frame) / static_cast<Precision>(landmarksSize);
    for (size_t i = 0; i < landmarksSize; ++i) {
      const Precision kPixelDepthRelativeDeviation = 2e-3_p;

      auto& landmark = odometry_track.lastKeyframe().getImmatureLandmark(sensor, i);
      const auto& coords = landmark.projection();
      Precision idepth = 1._p / (*gt_depth_frame)[static_cast<size_t>(coords(0))][static_cast<size_t>(coords(1))];
      if (add_noise) {
        idepth += static_cast<Precision>(rand() * 2.0 / RAND_MAX - 1) * kPixelDepthRelativeDeviation;
      }
      landmark.setIdepthMin(idepth);
      landmark.setIdepthMax(idepth);
      if (rand() * 1.0 / RAND_MAX <= landmark_pick_probability) {
        landmark.setStatus(track::landmarks::ImmatureStatus::kGood);
      } else {
        landmark.setStatus(track::landmarks::ImmatureStatus::kDelete);
      }
      landmark.setSearchPixelInterval(0);
    }

    landmarks_activator->activate(odometry_track);
  }
  for (size_t k = 0; k < odometry_track.activeFrames().size(); k++) {
    auto& keyframe = odometry_track.getActiveKeyframe(k);
    for (size_t idx = 0; idx < keyframe.activeLandmarks(sensor).size(); idx++) {
      keyframe.getActiveLandmark(sensor, idx).setIdepthVariance(2);
      keyframe.getActiveLandmark(sensor, idx).setRelativeBaseline(1);
    }
  }
  if (!add_connections) return;
  for (auto& reference_frame : odometry_track.activeFrames()) {
    for (auto& target_frame : odometry_track.activeFrames()) {
      if (reference_frame->keyframeId() >= target_frame->keyframeId()) continue;
      auto connection =
          std::make_unique<track::FrameConnection<Motion>>(reference_frame->keyframeId(), target_frame->keyframeId());
      connection->addSensorConnection(sensor, sensor, reference_frame->activeLandmarks(sensor).size(),
                                      target_frame->activeLandmarks(sensor).size());
      reference_frame->addConnection(target_frame->keyframeId(), connection.get());
      target_frame->addConnection(reference_frame->keyframeId(), connection.get());
      odometry_track.connections().add(std::move(connection));
    }
  }
}

template struct SolverTestData<energy::motion::SE3<Precision>, features::PixelMap, false>;
template struct SolverTestData<energy::motion::SE3<Precision>, features::CeresGrid, false>;

}  // namespace test_tools
}  // namespace dsopp
