#include "energy/problems/photometric_bundle_adjustment/ceres_photometric_bundle_adjustment.hpp"
#include "energy/problems/pose_alignment/eigen_pose_alignment.hpp"

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "common/settings.hpp"
#include "common/time/time.hpp"
#include "features/camera/ceres_grid.hpp"
#include "features/camera/pixel_data_frame.hpp"
#include "features/camera/pixel_map.hpp"
#include "features/camera/tracking_feature.hpp"
#include "sensor/synchronized_frame.hpp"
#include "sensors/camera/camera.hpp"
#include "test/tools/solver_test_data.hpp"
#include "test/tools/transformations_equality.hpp"
#include "track/active_track.hpp"
#include "tracker/create_depth_maps.hpp"

#define DEBUG_SHOW_DEPTH_MAPS COMPILE_HIDDEN_CODE

namespace dsopp {
namespace energy {
namespace problem {
TEST(reference_frame_depth_map, reference_frame_depth_map) {
  const Precision kErrorThreshold = 0.01_p;
  const Precision kMinPercentageOfPointsAboutIdealDepth = 0.50;

  using SE3 = motion::SE3<Precision>;
  using Model = test_tools::SolverTestData<SE3>::Model;
  test_tools::SolverTestData<SE3> data({0, 5, 10, 15, 20, 25, 30, 35, 40, 45});
  track::ActiveOdometryTrack<SE3> &odometry_track = data.track.odometryTrack();

  auto reference_frame =
      tracker::createReferenceDepthMaps<SE3, Model>(odometry_track.activeFrames(), data.camera->calibration());
  auto &depth_maps = reference_frame.at(data.sensor);

  Precision number_of_points = 0;
  Precision number_of_points_about_ideal_depth = 0;

  for (size_t lvl = 0; lvl < depth_maps.size(); lvl++) {
    auto &depth_map = depth_maps[lvl].map;
    long width = depth_map.rows(), height = depth_map.cols();
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        if (depth_map(x, y).weight > 1e-10) {
          Precision depth = 1 / (depth_map(x, y).idepth / depth_map(x, y).weight);
          auto x_global = static_cast<size_t>(x * (1 << lvl));
          auto y_global = static_cast<size_t>(y * (1 << lvl));
          Precision depth_gt = data.gt_depth_maps.at(odometry_track.lastKeyframe().id() - 10)->at(x_global)[y_global];
          if (std::abs((depth_gt - depth) / depth_gt) < kErrorThreshold) {
            number_of_points_about_ideal_depth++;
          }
          number_of_points++;
        }
      }
    }
  }

  Precision percentage_of_points_about_ideal_depth = number_of_points_about_ideal_depth / number_of_points;

  CHECK_GE(percentage_of_points_about_ideal_depth, kMinPercentageOfPointsAboutIdealDepth);

  std::cout << "Percentage of points about ideal depth: " << percentage_of_points_about_ideal_depth * 100 << std::endl;

#if DEBUG_SHOW_DEPTH_MAPS
  const Precision kMinDist = 1;
  const Precision kMaxDist = 200;

  for (size_t lvl = 0; lvl < depth_maps.size(); lvl++) {
    auto &depth_map = depth_maps[lvl].map;
    int width = static_cast<int>(depth_map.rows()), height = static_cast<int>(depth_map.cols());
    cv::Mat image(height, width, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        if (depth_map(x, y).weight > 0) {
          Precision depth = std::clamp(1 / (depth_map(x, y).idepth / depth_map(x, y).weight), kMinDist, kMaxDist);
          image.at<uint8_t>(y, x) = static_cast<uint8_t>((depth - kMinDist) * 255 / (kMaxDist - kMinDist));
        }
      }
    }
    cv::resize(image, image, cv::Size(), 1 << lvl, 1 << lvl);
    cv::imshow("Level " + std::to_string(lvl), image);
  }
  cv::waitKey();
#endif
}
}  // namespace problem
}  // namespace energy
}  // namespace dsopp
