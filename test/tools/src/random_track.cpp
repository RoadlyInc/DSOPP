#include "test/tools/random_track.hpp"

#include "track/connections/connections_container.hpp"
#include "track/connections/frame_connection.hpp"
#include "track/landmarks/tracking_landmark.hpp"
#include "track/odometry_track.hpp"
#include "track/track.hpp"

#include "mock_camera_provider.hpp"
#include "sensors/camera/camera.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"
#include "sensors/camera_calibration/camera_settings.hpp"
#include "sensors/camera_calibration/fabric.hpp"
#include "sensors/camera_calibration/mask/camera_mask.hpp"

#include <Eigen/Dense>
#include <random>

namespace dsopp {

namespace test_tools {
namespace {

std::map<size_t, sanity_checker::SanityCheckStatus> sanityCheckResults() {
  std::map<size_t, sanity_checker::SanityCheckStatus> sanity_check_results;
  sanity_check_results[0] = sanity_checker::SanityCheckStatus::kExceededGravityAngle;
  sanity_check_results[1] = sanity_checker::SanityCheckStatus::kExceededGravityAngularVelocity;
  sanity_check_results[2] = sanity_checker::SanityCheckStatus::kExceededRotationAngle;
  sanity_check_results[3] = sanity_checker::SanityCheckStatus::kExceededRotationAngularVelocity;
  sanity_check_results[4] = sanity_checker::SanityCheckStatus::kExceededTranslationError;
  return sanity_check_results;
}

sensors::calibration::CameraSettings createCameraSettings(sensors::calibration::CameraCalibration &&calib) {
  Eigen::Vector2<int> im_size = calib.image_size().cast<int>();

  cv::Mat image(im_size.x(), im_size.y(), CV_8UC3, cv::Scalar(255, 255, 255));
  image.at<cv::Vec3b>(50, 50) = cv::Vec3b(0, 0, 0);
  auto provider = std::make_unique<sensors::providers::MockCameraProvider>();
  auto photo_calib = sensors::calibration::photometric_calibration::create(TEST_DATA_DIR "track30seconds/pcalib.txt");
  auto vignetting = sensors::calibration::vignetting::create(TEST_DATA_DIR "track30seconds/vignetting.png");
  std::ifstream legend_file(TEST_DATA_DIR "track30seconds/semantic_legend.txt");
  auto legend = std::make_unique<semantics::SemanticLegend>(legend_file);

  auto camera_mask = sensors::calibration::CameraMask(im_size.y(), im_size.x());

  return sensors::calibration::CameraSettings(std::move(calib), std::move(photo_calib), std::move(vignetting),
                                              std::move(camera_mask), std::move(legend));
}

std::unique_ptr<sensors::AgentSettings> agentSettings() {
  auto agent_settings = std::make_unique<sensors::AgentSettings>();
  const Precision shutterTimeInSeconds = 0.1_p;
  const auto kShutterTime =
      std::chrono::duration_cast<time::duration>(std::chrono::duration<Precision>(shutterTimeInSeconds));
  /** Add Pinhole sensor */
  {
    auto calib = sensors::calibration::CameraCalibration(
        Eigen::Vector2<Precision>(200, 200), Eigen::Vector4<Precision>(100, 100, 200, 200),
        energy::model::ModelType::kPinholeCamera, kShutterTime,
        sensors::calibration::Undistorter::Identity(Eigen::Vector2<Precision>(200, 200)));
    auto settings = createCameraSettings(std::move(calib));
    agent_settings->addCameraSettings(std::move(settings));
  }
  /** Add Simple Radial sensor */
  {
    auto calib = sensors::calibration::CameraCalibration(
        Eigen::Vector2<Precision>(200, 200), Eigen::Vector<Precision, 5>(100, 100, 200, 200, 100),
        energy::model::ModelType::kSimpleRadialCamera, kShutterTime,
        sensors::calibration::Undistorter::Identity(Eigen::Vector2<Precision>(200, 200)));
    auto settings = createCameraSettings(std::move(calib));
    agent_settings->addCameraSettings(std::move(settings));
  }

  return agent_settings;
}
}  // namespace

std::unique_ptr<track::OdometryTrack<energy::motion::SE3<Precision>>> randomTrack(int offset) {
  using SE3 = energy::motion::SE3<Precision>;
  size_t sensor_id = 0;
  const size_t kFramesNumber = 600;
  const size_t kPointsNumber = 1000;
  const size_t kConnectionsNum = 8;

  std::random_device random_device;
  std::mt19937 generator(random_device());
  std::uniform_real_distribution<Precision> random_distribution(0, 1);

  std::vector<std::unique_ptr<track::Keyframe<SE3>>> frames;
  const auto start = std::chrono::high_resolution_clock::now();
  for (size_t frame_index = 0; frame_index < kFramesNumber; ++frame_index) {
    const auto t = time(std::chrono::high_resolution_clock::now() - start);
    track::Keyframe<SE3>::LandmarksFrame landmarks;
    landmarks[sensor_id].reserve(kPointsNumber);
    for (size_t point_index = 0; point_index < kPointsNumber; point_index++) {
      Eigen::Vector3<Precision> direction(random_distribution(generator) * 10, random_distribution(generator) * 10, 1);
      Eigen::Vector2<Precision> proj(random_distribution(generator) * 10, random_distribution(generator) * 10);
      landmarks[sensor_id].emplace_back(track::landmarks::TrackingLandmark(
          direction, proj, random_distribution(generator) * 10, 0, random_distribution(generator)));
    }
    auto frame = std::make_unique<track::Keyframe<SE3>>(
        frame_index, frame_index, t,
        Sophus::SE3<Precision>::exp(Sophus::SE3<Precision>::Tangent(static_cast<Precision>(offset), 0,
                                                                    -static_cast<Precision>(frame_index), 0, 0, 0)),
        Eigen::Vector2<Precision>::Random(), std::move(landmarks));
    frames.push_back(std::move(frame));
  }
  auto connections = std::make_unique<track::ConnectionsContainer<SE3>>();
  for (size_t frame_index = 0; frame_index < kFramesNumber; ++frame_index) {
    for (size_t reference_frame_id = frame_index > kConnectionsNum ? frame_index - kConnectionsNum : 0;
         reference_frame_id < frame_index; reference_frame_id++) {
      auto connection = std::make_unique<track::FrameConnection<SE3>>(reference_frame_id, frame_index);
      connection->setCovariance(Eigen::Matrix<Precision, SE3::DoF, SE3::DoF>::Random());
      connections->add(std::move(connection));
    }
  }
  return std::make_unique<track::OdometryTrack<SE3>>(std::move(frames), std::move(connections));
}

std::unique_ptr<track::Track<energy::motion::SE3<Precision>>> track() {
  return std::make_unique<track::Track<energy::motion::SE3<Precision>>>(
      test_tools::agentSettings(), test_tools::randomTrack(), test_tools::sanityCheckResults());
}

}  // namespace test_tools
}  // namespace dsopp
