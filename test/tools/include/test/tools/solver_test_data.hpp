#ifndef DSOPP_SOLVER_TEST_DATA_HPP
#define DSOPP_SOLVER_TEST_DATA_HPP

#include <memory>

#include "depth_gt.hpp"
#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/motion/motion.hpp"
#include "features/camera/camera_features.hpp"
#include "features/camera/pattern_patch.hpp"
#include "sensors/camera/camera.hpp"

#include "test/tools/tum_gt.hpp"
#include "track/active_odometry_track.hpp"
#include "track/active_track.hpp"

namespace dsopp {
namespace test_tools {
template <energy::motion::Motion Motion, template <int> typename Grid2D = features::PixelMap,
          bool GN_NET_EMBEDDER = false>
struct SolverTestData {
  using Calibration = sensors::calibration::CameraCalibration;
  using Model = energy::model::PinholeCamera<Precision>;
  static constexpr energy::model::ModelType ModelType = energy::model::ModelType::kPinholeCamera;
  SolverTestData(std::vector<size_t> frame_ids, bool add_noise = false, int max_points_per_frame = 800,
                 bool add_connections = true);
  std::unique_ptr<sensors::calibration::CameraSettings> settings;
  std::unique_ptr<sensors::Camera> camera;
  std::unique_ptr<Model> model;
  track::ActiveTrack<Motion> track;
  size_t sensor = 0;
  std::map<size_t, Motion> gt_cam;
  std::map<size_t, std::shared_ptr<test_tools::DepthGt::Frame>> gt_depth_maps;

  test_tools::TumGt gt;
};
}  // namespace test_tools
}  // namespace dsopp

#endif  // DSOPP_SOLVER_TEST_DATA_HPP
