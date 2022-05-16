#include <benchmark/benchmark.h>

#include <functional>
#include <memory>
#include <numeric>
#include <vector>

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
#include "features/camera/tracking_feature.hpp"
#include "sensors/camera/camera.hpp"
#include "sensors/camera_calibration/fabric.hpp"

#include "energy/epipolar_geometry/epipolar_line_builder.hpp"
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

using namespace dsopp;
using time_point = std::chrono::system_clock::time_point;
using SE3 = dsopp::energy::motion::SE3<Precision>;
using Calibration = sensors::calibration::CameraCalibration;
using Model = energy::model::PinholeCamera<Precision>;

namespace {
struct BenchmarkData {
  BenchmarkData() {
    size_t current_frame_id = 0;
    data = std::make_unique<test_tools::SolverTestData<SE3>>(std::vector<size_t>{}, false, 1500);

    auto& camera = data->camera;
    camera_model = camera->calibration().cameraModel<Model>();
    auto& odometry_track = data->track.odometryTrack();

    for (int i = 0; i < 50; ++i) {
      sensors::SynchronizedFrame synchronized_frame(camera->nextFrameId(), camera->nextFrameTime());
      camera->processNextDataFrame(synchronized_frame);
    }

    {
      sensors::SynchronizedFrame synchronized_frame(camera->nextFrameId(), camera->nextFrameTime());
      camera->processNextDataFrame(synchronized_frame);
      auto& features = *synchronized_frame.cameraFeatures().at(0);

      SE3 gt_world_cam1 = SE3(data->gt.getPose(features.timestamp()));
      odometry_track.pushFrame(current_frame_id++, features.timestamp(), gt_world_cam1);
      {
        auto pyramid = features.movePixelData();
        odometry_track.lastKeyframe().pushPyramid(0, features::movePyramidAndDelete(pyramid));
      }
      odometry_track.lastKeyframe().pushImmatureLandmarks(0,
                                                          tracker::buildFeatures(features.tracking(), *camera_model));
    }

    const size_t immature_landmarks_size = odometry_track.lastKeyframe().immatureLandmarks(0).size();

    sensors::SynchronizedFrame synchronized_frame(camera->nextFrameId(), camera->nextFrameTime());
    camera->processNextDataFrame(synchronized_frame);
    auto& features = *synchronized_frame.cameraFeatures().at(0);

    const auto& gt_world_agent = SE3(data->gt.getPose(features.timestamp()));
    t_t_r = odometry_track.lastKeyframe().tWorldAgent().inverse() * gt_world_agent;
    target_grid = std::make_unique<features::PixelMap<1>>(features.pixelData().getLevel(0).clone());

    for (size_t landmark_i = 0; landmark_i < immature_landmarks_size; ++landmark_i) {
      auto& landmark = odometry_track.lastKeyframe().getImmatureLandmark(0, landmark_i);
      pure_landmarks.emplace_back(&landmark);
    }
  }

  std::vector<std::reference_wrapper<track::landmarks::ImmatureTrackingLandmark>>& getLandmarks() {
    bad_landmarks.clear();
    landmarks.clear();
    for (auto& landmark : pure_landmarks) bad_landmarks.emplace_back(landmark);

    for (auto& landmark : bad_landmarks) landmarks.push_back(*landmark);
    return landmarks;
  }

  std::unique_ptr<test_tools::SolverTestData<SE3>> data;

  std::vector<std::reference_wrapper<track::landmarks::ImmatureTrackingLandmark>> landmarks;
  std::vector<track::landmarks::ImmatureTrackingLandmark*> pure_landmarks, bad_landmarks;

  std::unique_ptr<features::PixelMap<1>> target_grid;
  SE3 t_t_r;
  std::unique_ptr<Model> camera_model;
};

}  // namespace

static void DepthEstimatorBehcnmark(benchmark::State& state) {
  BenchmarkData data;
  for (auto _ : state) {
    state.PauseTiming();
    auto& landmarks = data.getLandmarks();
    state.ResumeTiming();
    tracker::DepthEstimation::estimate<SE3, features::PixelMap, Model, 1>(
        *data.target_grid, landmarks, data.t_t_r, Eigen::Vector2<Precision>::Zero(), Eigen::Vector2<Precision>::Zero(),
        data.data->camera->calibration(), data.data->camera->pyramidOfMasks()[0], 9);
  }
}
BENCHMARK(DepthEstimatorBehcnmark)->Unit(benchmark::kMillisecond);

static void EpipolarLineBenchmark(benchmark::State& state) {
  BenchmarkData data;
  for (auto _ : state) {
    for (auto& landmark : data.pure_landmarks) {
      auto epiline_builder = energy::epipolar_geometry::EpipolarLineBuilder<Model, SE3>(*data.camera_model, data.t_t_r);
      auto epiline = epiline_builder.buildSegment(landmark->projection(), landmark->idepthMin(), landmark->idepthMax());
      const auto& epiline_points = epiline.points;
      benchmark::DoNotOptimize(epiline_points);
    }
  }
}
BENCHMARK(EpipolarLineBenchmark)->Unit(benchmark::kMillisecond);
