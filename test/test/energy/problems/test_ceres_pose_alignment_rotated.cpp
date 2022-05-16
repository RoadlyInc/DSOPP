#include <algorithm>
#include <fstream>
#include <numbers>
#include <random>
#include <string>

#include <gmock/gmock.h>

#include "common/settings.hpp"
#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"
#include "energy/problems/photometric_bundle_adjustment/ceres_photometric_bundle_adjustment.hpp"
#include "energy/problems/photometric_bundle_adjustment/photometric_bundle_adjustment.hpp"
#include "energy/problems/pose_alignment/pose_alignment.hpp"
#include "features/camera/camera_features.hpp"
#include "features/camera/ceres_grid.hpp"
#include "features/camera/frame_embedding_extractor.hpp"
#include "features/camera/pattern_patch.hpp"
#include "features/camera/pixel_data_frame.hpp"
#include "features/camera/pixel_map.hpp"
#include "features/camera/sobel_tracking_features_extractor.hpp"
#include "features/camera/tracking_feature.hpp"
#include "mock_camera_provider.hpp"
#include "sensors/camera/camera.hpp"
#include "sensors/camera_calibration/fabric.hpp"
#include "sensors/camera_calibration/mask/camera_mask.hpp"

#include "sensor/synchronized_frame.hpp"
#include "sensors/camera_providers/image_folder_provider.hpp"
#include "test/tools/depth_gt.hpp"
#include "test/tools/solver_test_data.hpp"
#include "test/tools/transformations_equality.hpp"
#include "test/tools/tum_gt.hpp"
#include "track/active_odometry_track.hpp"
#include "track/frames/active_keyframe.hpp"
#include "track/landmarks/active_tracking_landmark.hpp"
#include "tracker/build_features.hpp"
#include "tracker/depth_estimators/depth_estimation.hpp"
#include "tracker/landmarks_activator/landmarks_activator.hpp"

using timeC = std::chrono::time_point<std::chrono::high_resolution_clock>;

using namespace dsopp;

namespace {
/** Read Image to World pose */
Sophus::SE3<Precision> readPose(const std::string &file) {
  std::ifstream f(file);
  CHECK(f.is_open());

  Eigen::Matrix<Precision, 4, 4, Eigen::RowMajor> pose;
  pose.setIdentity();

  for (int i = 0; i < 12; ++i) f >> pose.data()[i];
  return Sophus::SE3<Precision>(pose);
}

Eigen::Matrix<Precision, -1, -1, Eigen::RowMajor> readDepth(const std::string &file, const int H, const int W) {
  Eigen::Matrix<Precision, -1, -1, Eigen::RowMajor> out(H, W);

  std::ifstream f(file);
  CHECK(f.is_open());
  for (int i = 0; i < W * H; ++i) f >> out.data()[i];
  return out;
}

}  // namespace

class CeresPoseAlignmentTest : public ::testing::Test {
 public:
  using Calibration = sensors::calibration::CameraCalibration;
  using Model = energy::model::PinholeCamera<Precision>;
  static constexpr energy::model::ModelType ModelType = energy::model::ModelType::kPinholeCamera;
  using SE3 = energy::motion::SE3<Precision>;

  void SetUp() {
    std::string frame1 = TEST_DATA_DIR "rotation_pair/img1.png";
    std::string frame2 = TEST_DATA_DIR "rotation_pair/img2.png";
    std::string pose1_file = TEST_DATA_DIR "rotation_pair/pose_img1.txt";
    std::string pose2_file = TEST_DATA_DIR "rotation_pair/pose_img2.txt";
    std::string depth_img1 = TEST_DATA_DIR "rotation_pair/depth_img1.png.npy";

    cv::Mat img1 = cv::imread(frame1);
    cv::Mat img2 = cv::imread(frame2);

    EXPECT_FALSE(img1.empty());
    EXPECT_FALSE(img2.empty());

    H = img1.rows;
    W = img1.cols;

    Sophus::SE3<Precision> t_world_pose1 = readPose(pose1_file);
    Sophus::SE3<Precision> t_world_pose2 = readPose(pose2_file);
    gt_pose1_pose2 = t_world_pose1.inverse() * t_world_pose2;

    Eigen::MatrixX<Precision> depth = readDepth(depth_img1, H, W);

    auto data_frame1 = new sensors::providers::CameraDataFrame(0, std::move(img1), timeC(std::chrono::milliseconds(0)));
    auto data_frame2 = new sensors::providers::CameraDataFrame(1, std::move(img2), timeC(std::chrono::milliseconds(1)));

    auto provider = std::make_unique<sensors::providers::MockCameraProvider>();

    int counter = 0;

    EXPECT_CALL(*provider, nextFrameProxy()).WillRepeatedly([&]() noexcept {
      if (counter == 0) {
        counter++;
        return data_frame1;
      } else
        return data_frame2;
    });

    auto photo_calib = sensors::calibration::photometric_calibration::create(TEST_DATA_DIR "rotation_pair/pcalib.txt");
    auto vignetting = sensors::calibration::vignetting::create(TEST_DATA_DIR "rotation_pair/vignetting.png");

    auto calib =
        Calibration(Eigen::Vector2<Precision>(512, 512), Eigen::Vector4<Precision>(256, 256, 256, 256), ModelType);
    model = calib.cameraModel<Model>();
    Eigen::Vector2<int> im_size = calib.image_size().cast<int>();

    auto camera_mask = sensors::calibration::CameraMask(im_size.y(), im_size.x());

    settings = std::make_unique<sensors::calibration::CameraSettings>(std::move(calib), std::move(photo_calib),
                                                                      std::move(vignetting), std::move(camera_mask));
    camera = std::make_unique<sensors::Camera>("camera_1", 0, *settings, std::move(provider),
                                               std::make_unique<dsopp::features::SobelTrackingFeaturesExtractor>(1500));

    synchronized_frames.push_back(
        std::make_unique<sensors::SynchronizedFrame>(camera->nextFrameId(), camera->nextFrameTime()));
    camera->processNextDataFrame(*synchronized_frames[0]);
    features.push_back(synchronized_frames[0]->cameraFeatures().at(sensor).get());
    synchronized_frames.push_back(
        std::make_unique<sensors::SynchronizedFrame>(camera->nextFrameId(), camera->nextFrameTime()));
    camera->processNextDataFrame(*synchronized_frames[1]);
    features.push_back(synchronized_frames[1]->cameraFeatures().at(sensor).get());

    auto landmarks_activator =
        std::make_unique<tracker::LandmarksActivator<SE3, Model, dsopp::features::PixelMap, 1>>(camera->calibration());

    for (size_t idx = 0; idx < 2; idx++) {
      track.pushFrame(idx, features[idx]->timestamp(), Sophus::SE3<Precision>());
      {
        auto pyramids = features[idx]->movePixelData();
        track.lastKeyframe().pushPyramid(sensor, features::movePyramidAndDelete(pyramids));
        track.lastKeyframe().pushPyramidOfMasks(sensor, features[idx]->movePyramidOfMasks());
      }
      track.lastKeyframe().pushImmatureLandmarks(sensor, tracker::buildFeatures(features[idx]->tracking(), *model));
      {
        auto pyramids = features[idx]->movePixelData();
        pyramids_[sensor] = features::movePyramidAndDelete(pyramids);
      }

      const size_t landmarksSize = track.lastKeyframe().immatureLandmarks(sensor).size();
      for (size_t i = 0; i < landmarksSize; ++i) {
        auto &landmark = track.lastKeyframe().getImmatureLandmark(sensor, i);
        const auto &coords = landmark.projection();
        Precision idepth = 1._p / depth(int(coords(1)), int(coords(0)));
        landmark.setIdepthMin(idepth);
        landmark.setIdepthMax(idepth);
        landmark.setStatus(track::landmarks::ImmatureStatus::kGood);
        landmark.setSearchPixelInterval(0);
      }
      landmarks_activator->activate(track);
    }
  }

 protected:
  const size_t kMaxIteration = 50;
  const Precision kInitialTrustRegionRadius = 1e2;
  const Precision kFunctionTolerance = 1e-8_p;
  const Precision kParameterTolerance = 1e-8_p;
  std::unique_ptr<sensors::calibration::CameraSettings> settings;
  std::unique_ptr<sensors::Camera> camera;
  Sophus::SE3<Precision> gt_pose1_pose2;
  track::ActiveOdometryTrack<SE3> track;
  track::ActiveKeyframe<SE3>::Pyramids pyramids_;
  std::unique_ptr<energy::model::PinholeCamera<Precision>> model;

  std::vector<std::unique_ptr<sensors::SynchronizedFrame>> synchronized_frames;
  std::vector<features::CameraFeatures *> features;

  const Precision distance_eps = 1e-1_p;
  const Precision angle_eps = 1 * std::numbers::pi_v<Precision> / 180.0_p;

  size_t sensor = 0;
  std::vector<track::landmarks::ImmatureTrackingLandmark> *landmarks;
  int W, H;
  Precision kHuberLossSigma = 15;

  template <class Solver>
  Sophus::SE3<Precision> solve(Solver &solver, const Sophus::SE3<Precision> &init, size_t level) {
    solver.pushFrame(track.getActiveKeyframe(0), level, *model, energy::problem::FrameParameterization::kFixed);
    solver.pushFrame(features[1]->timestamp(), init, pyramids_, {{sensor, camera->pyramidOfMasks()[0]}},
                     Eigen::Vector2<Precision>::Zero(), level, *model);
    solver.solve(4);
    auto t_cam1_cam2 = track.getActiveKeyframe(0).tWorldAgent().inverse() * solver.getPose(features[1]->timestamp());
    return t_cam1_cam2;
  }
  void checkDSOPP(const Sophus::SE3<Precision> &init) {
    Sophus::SE3<Precision> t_cam1_cam2 = init;
    for (int i = static_cast<int>(pyramids_.size()) - 1; i >= 0; --i) {
      auto dsopp_solver = energy::problem::CeresPhotometricBundleAdjustment<SE3, Model, Pattern::kSize,
                                                                            features::PixelMap, true, false>(
          energy::problem::TrustRegionPhotometricBundleAdjustmentOptions<double>(
              kMaxIteration, kInitialTrustRegionRadius, kFunctionTolerance, kParameterTolerance,
              Eigen::Vector2<double>::Constant(1e12), 1e16, kHuberLossSigma),
          false, true, static_cast<size_t>(i));

      t_cam1_cam2 = solve(dsopp_solver, t_cam1_cam2, static_cast<size_t>(i));
    }
    assertEqual(gt_pose1_pose2, t_cam1_cam2, distance_eps, angle_eps);
  }

  void checkCeres(const Sophus::SE3<Precision> &init) {
    Sophus::SE3<Precision> t_cam1_cam2 = init;
    for (int i = static_cast<int>(pyramids_.size()) - 1; i >= 0; --i) {
      auto ceres_solver = energy::problem::CeresPhotometricBundleAdjustment<SE3, Model, Pattern::kSize,
                                                                            features::CeresGrid, true, false>(
          energy::problem::TrustRegionPhotometricBundleAdjustmentOptions<double>(
              kMaxIteration, kInitialTrustRegionRadius, kFunctionTolerance, kParameterTolerance,
              Eigen::Vector2<double>::Constant(1e12), 1e16, kHuberLossSigma),
          false, true, static_cast<size_t>(i));

      t_cam1_cam2 = solve(ceres_solver, t_cam1_cam2, static_cast<size_t>(i));
    }
    assertEqual(gt_pose1_pose2, t_cam1_cam2, distance_eps, angle_eps);
  }
};

TEST_F(CeresPoseAlignmentTest, DSOPPGridFromGT) {
  Sophus::SE3<Precision> init = gt_pose1_pose2;
  checkDSOPP(init);
}

TEST_F(CeresPoseAlignmentTest, DSOPPGridFromNoisyRotationGT) {
  std::mt19937 rng;
  /** Approx 0.5 degree */
  std::uniform_real_distribution<Precision> x(-0.01_p, 0.01_p);
  Sophus::SE3<Precision> init = gt_pose1_pose2;
  init.so3() = init.so3() * Sophus::SO3<Precision>::exp(Eigen::Vector3<Precision>(x(rng), x(rng), x(rng)));
  checkDSOPP(init);
}

TEST_F(CeresPoseAlignmentTest, DSOPPGridFromNoisyTranslationGT) {
  std::mt19937 rng;

  Precision norm = gt_pose1_pose2.translation().norm();
  std::uniform_real_distribution<Precision> x(-0.1_p * norm, 0.1_p * norm);

  Sophus::SE3<Precision> init = gt_pose1_pose2;
  init.translation() = init.translation() + Eigen::Vector3<Precision>(x(rng), x(rng), x(rng));

  checkDSOPP(init);
}

TEST_F(CeresPoseAlignmentTest, DSOPPGridFromNoisyGT) {
  std::mt19937 rng;

  Precision norm = gt_pose1_pose2.translation().norm();
  std::uniform_real_distribution<Precision> x(-0.05_p * norm, 0.05_p * norm);
  std::uniform_real_distribution<Precision> r(-0.01_p, 0.01_p);

  Sophus::SE3<Precision> init = gt_pose1_pose2;
  init.translation() = init.translation() + Eigen::Vector3<Precision>(x(rng), x(rng), x(rng));
  init.so3() = init.so3() * Sophus::SO3<Precision>::exp(Eigen::Vector3<Precision>(x(rng), x(rng), x(rng)));

  checkDSOPP(init);
}

TEST_F(CeresPoseAlignmentTest, CeresGridFromGT) {
  Sophus::SE3<Precision> init = gt_pose1_pose2;
  checkCeres(init);
}

TEST_F(CeresPoseAlignmentTest, CeresGridFromNoisyRotationGT) {
  std::mt19937 rng;
  /** Approx 0.5 degree */
  std::uniform_real_distribution<Precision> x(-0.01_p, 0.01_p);

  Sophus::SE3<Precision> init = gt_pose1_pose2;
  init.so3() = init.so3() * Sophus::SO3<Precision>::exp(Eigen::Vector3<Precision>(x(rng), x(rng), x(rng)));

  checkCeres(init);
}

TEST_F(CeresPoseAlignmentTest, CeresGridFromNoisyTranslationGT) {
  std::mt19937 rng;

  Precision norm = gt_pose1_pose2.translation().norm();
  std::uniform_real_distribution<Precision> x(-0.1_p * norm, 0.1_p * norm);

  Sophus::SE3<Precision> init = gt_pose1_pose2;
  init.translation() = init.translation() + Eigen::Vector3<Precision>(x(rng), x(rng), x(rng));

  checkCeres(init);
}

TEST_F(CeresPoseAlignmentTest, CeresGridFromNoisyGT) {
  std::mt19937 rng;

  Precision norm = gt_pose1_pose2.translation().norm();
  std::uniform_real_distribution<Precision> x(-0.01_p * norm, 0.01_p * norm);
  std::uniform_real_distribution<Precision> r(-0.005_p, 0.051_p);

  Sophus::SE3<Precision> init = gt_pose1_pose2;
  init.translation() = init.translation() + Eigen::Vector3<Precision>(x(rng), x(rng), x(rng));
  init.so3() = init.so3() * Sophus::SO3<Precision>::exp(Eigen::Vector3<Precision>(x(rng), x(rng), x(rng)));

  checkCeres(init);
}
