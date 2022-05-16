#include <random>

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "common/settings.hpp"
#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/motion/se3_motion.hpp"
#include "energy/problems//geometric_bundle_adjustment/ceres_geometric_bundle_adjustment.hpp"
#include "energy/problems//geometric_bundle_adjustment/local_frame.hpp"

namespace dsopp::energy::problem::geometric_bundle_adjustment {
class GeometricBundleAdjustmentTest : public ::testing::Test {
 public:
  void SetUp() {
    image_size_ = Eigen::Vector2<Precision>(1280, 640);
    intrinsics_ = Eigen::Vector4<Precision>(448, 448, 640, 320);
    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<Precision> pixel_noise(-30, 30);

    for (int i = 0; i < 4; ++i) {
      intrinsics_noised_(i) = intrinsics_(i) + pixel_noise(rng);
    }

    while (!generateFrames())
      ;
  }

  bool generateFrames() {
    frame_points_.clear();
    frame_points_.resize(kFrameN);
    points3d_.clear();
    t_a_w_ = {motion::SE3<double>()};

    model::PinholeCamera<double> camera(image_size_, intrinsics_.cast<double>().eval());
    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> angle(-M_PI / 6, M_PI / 6);
    std::uniform_real_distribution<double> coord(-5, 5);
    std::uniform_real_distribution<double> z_coord(0, 15);

    for (size_t i = 1; i < kFrameN; ++i) {
      Eigen::Vector3d so3_log(angle(rng), angle(rng), angle(rng));
      Eigen::Vector3d trans(coord(rng), coord(rng), abs(coord(rng)));
      t_a_w_.emplace_back(Sophus::SE3d(Sophus::SO3d::exp(so3_log), trans));
    }

    std::vector<int> success_projections(kFrameN, 0);

    const int kMinProjectionFrames = 2;
    for (size_t i = 0; i < kPointsN; ++i) {
      Eigen::Vector3d point3d(coord(rng), coord(rng), z_coord(rng));

      Eigen::Vector2d pixel;
      int projections = 0;

      for (size_t frame = 0; frame < kFrameN; ++frame) {
        Eigen::Vector3d frame_point3d = t_a_w_[frame] * point3d;
        if (camera.project(frame_point3d, pixel)) projections++;
      }

      if (projections >= kMinProjectionFrames) {
        points3d_.push_back(point3d);
        for (size_t frame = 0; frame < kFrameN; ++frame) {
          Eigen::Vector3d frame_point3d = t_a_w_[frame] * point3d;

          if (camera.project(frame_point3d, pixel)) {
            frame_points_[frame].push_back(pixel);

            success_projections[frame]++;
          } else {
            frame_points_[frame].push_back(std::nullopt);
          }
        }
      }
    }
    return *std::min(success_projections.begin(), success_projections.end()) > kMinProjected;
  }

  void TearDown() {}

 protected:
  Eigen::Vector2<Precision> image_size_;
  Eigen::Vector4<Precision> intrinsics_;
  Eigen::Vector4<Precision> intrinsics_noised_;

  const int kThreads = 1;
  const int kPixelThreshold = 2;

  const size_t kFrameN = 3;
  const size_t kPointsN = 100;
  const int kMinProjected = 40;
  std::vector<Eigen::Vector3<double>> points3d_;

  std::vector<motion::SE3<double>> t_a_w_;
  std::vector<std::vector<std::optional<Eigen::Vector2d>>> frame_points_;
};

auto runBundler(std::vector<LocalFrame> &frames, const Eigen::Vector4<Precision> &intrinsics,
                const Eigen::Vector2<Precision> &image_size, const int threads, const int threshold) {
  using Calibration = sensors::calibration::CameraCalibration;
  using Model = energy::model::PinholeCamera<Precision>;
  Calibration calibration(image_size, intrinsics, energy::model::ModelType::kPinholeCamera);
  auto model = calibration.cameraModel<Model>();
  energy::problem::geometric_bundle_adjustment::CeresGeometricBundleAdjustmentSolver<Model> solver(
      model->image_size(), model->intrinsicsParameters(), threads, threshold);
  energy::problem::geometric_bundle_adjustment::ParameterParameterization parameterization;
  parameterization.fix_focal = false;
  parameterization.fix_center = false;
  parameterization.fix_model_specific = false;
  solver.solve(frames, parameterization);
  return solver.camera_intrinsics();
}

TEST_F(GeometricBundleAdjustmentTest, NoNoiseTest) {
  std::vector<LocalFrame> frames;
  for (size_t frame = 0; frame < kFrameN; ++frame) {
    frames.emplace_back(frame, t_a_w_[frame]);
    for (size_t i = 0; i < points3d_.size(); ++i) {
      if (frame_points_[frame][i]) {
        frames.back().pushObservation(*frame_points_[frame][i], points3d_[i]);
      }
    }
  }

  auto intrinsics_out = runBundler(frames, intrinsics_, image_size_, kThreads, kPixelThreshold);
  EXPECT_LE((intrinsics_ - intrinsics_out.cast<Precision>()).norm(), 1e-4);
}

TEST_F(GeometricBundleAdjustmentTest, NoiseTest_GT_Poses) {
  std::vector<LocalFrame> frames;
  for (size_t frame = 0; frame < kFrameN; ++frame) {
    frames.emplace_back(frame, t_a_w_[frame]);
    for (size_t i = 0; i < points3d_.size(); ++i) {
      if (frame_points_[frame][i]) {
        frames.back().pushObservation(*frame_points_[frame][i], points3d_[i]);
      }
    }
  }

  auto intrinsics_out = runBundler(frames, intrinsics_noised_, image_size_, kThreads, kPixelThreshold);
  EXPECT_LE((intrinsics_ - intrinsics_out.cast<Precision>()).norm(), 1);
}

TEST_F(GeometricBundleAdjustmentTest, NoiseTest_Noised_Poses) {
  std::mt19937 rng(std::random_device{}());
  std::uniform_real_distribution<double> angle_noise(-M_PI / 50, M_PI / 50);
  std::uniform_real_distribution<double> coord_noise(-0.3, 0.3);

  std::vector<LocalFrame> frames;
  for (size_t frame = 0; frame < kFrameN; ++frame) {
    Eigen::Vector3d so3_log(angle_noise(rng), angle_noise(rng), angle_noise(rng));
    Eigen::Vector3d trans(coord_noise(rng), coord_noise(rng), abs(coord_noise(rng)));
    Sophus::SE3d noise_delta(Sophus::SO3d::exp(so3_log), trans);

    frames.emplace_back(frame, t_a_w_[frame] * noise_delta);
    for (size_t i = 0; i < points3d_.size(); ++i) {
      if (frame_points_[frame][i]) {
        frames.back().pushObservation(*frame_points_[frame][i], points3d_[i]);
      }
    }
  }

  auto intrinsics_out =
      runBundler(frames, intrinsics_noised_, image_size_, kThreads, kPixelThreshold).cast<Precision>();

  using Calibration = sensors::calibration::CameraCalibration;
  using Model = energy::model::PinholeCamera<Precision>;
  Calibration calibration(image_size_, intrinsics_out, energy::model::ModelType::kPinholeCamera);
  auto model = calibration.cameraModel<Model>();

  for (size_t frame = 0; frame < kFrameN; ++frame) {
    auto t_a_w = frames[frame].t_agent_world;
    for (size_t i = 0; i < points3d_.size(); ++i) {
      if (frame_points_[frame][i]) {
        Eigen::Vector3d local_point3d = t_a_w * points3d_[i];
        Eigen::Vector2d pixel;

        if (model->project(local_point3d, pixel)) {
          Eigen::Vector2d diff = pixel - *frame_points_[frame][i];
          EXPECT_LE(diff.norm(), kPixelThreshold);
        }
      }
    }
  }
}

}  // namespace dsopp::energy::problem::geometric_bundle_adjustment
