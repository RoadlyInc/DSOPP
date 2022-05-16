#include "common/settings.hpp"
#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/motion/se3_motion.hpp"
#include "energy/problems/cost_functors/bundle_adjustment_photometric_cost_functor.hpp"
#include "energy/problems/cost_functors/bundle_adjustment_photometric_cost_functor_analytic.hpp"
#include "energy/problems/depth_map.hpp"
#include "energy/problems/photometric_bundle_adjustment/bundle_adjustment_photometric_evaluation_callback.hpp"
#include "features/camera/camera_features.hpp"
#include "features/camera/pixel_data_frame.hpp"
#include "sensors/camera/camera.hpp"
#include "sensors/camera_calibration/fabric.hpp"
#include "sensors/camera_calibration/mask/camera_mask.hpp"

#include "sensor/synchronized_frame.hpp"
#include "sensors/camera_providers/image_video_provider.hpp"
#include "test/tools/solver_test_data.hpp"
#include "track/landmarks/immature_tracking_landmark.hpp"

// break in and party
#include "energy/problems/photometric_bundle_adjustment/bundle_adjustment_photometric_evaluation_callback.hpp"
#include "energy/problems/photometric_bundle_adjustment/first_estimate_jacobians.hpp"

#include <ceres/jet.h>
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <sophus/se3.hpp>

namespace dsopp {
namespace energy {
namespace problem {
namespace {
template <int C>
std::unique_ptr<std::map<size_t, std::vector<features::PixelMap<C>>>> pyramids(
    const track::ActiveKeyframe<motion::SE3<Precision>>& frame);

template <>
std::unique_ptr<std::map<size_t, std::vector<features::PixelMap<1>>>> pyramids(
    const track::ActiveKeyframe<motion::SE3<Precision>>& frame) {
  auto frame_pyramids = std::make_unique<std::map<size_t, std::vector<features::PixelMap<1>>>>();
  for (const auto& [sensor, pyramid] : frame.pyramids()) {
    (*frame_pyramids)[sensor] = std::vector<features::PixelMap<1>>();
    for (auto& level : pyramid) {
      (*frame_pyramids)[sensor].emplace_back(level.clone());
    }
  }
  return frame_pyramids;
}

template <int C>
void testAnalyticalDiff() {
  using SE3 = motion::SE3<Precision>;
  using SE3d = motion::SE3<double>;
  const int ModelDof = test_tools::SolverTestData<SE3>::Model::DoF;
  std::vector<size_t> keyframe_ids({0, 5});
  test_tools::SolverTestData<SE3> data(keyframe_ids, false, 5);
  auto& frame1 = data.track.odometryTrack().getActiveKeyframe(0);
  auto& frame2 = data.track.odometryTrack().getActiveKeyframe(1);
  using Grid2D = typename features::PixelMap<C>;
  using CostFunctorAnalytic = cost_functors::BundleAdjustmentPhotometricCostFunctorAnalytic<SE3, Pattern::kSize, C>;
  using CostFunctorAuto = typename cost_functors::BundleAdjustmentPhotometricCostFunctor<
      SE3d, Grid2D, typename test_tools::SolverTestData<SE3>::Model, Pattern::kSize, C>;

  Eigen::Vector2<Precision> projection(500, 400);
  Eigen::Matrix<Precision, Pattern::kSize, C, PatchStorageOrder<C>> patch;
  Eigen::Matrix<Precision, 2, Pattern::kSize> pattern;
  features::PatternPatch::getIntensities(projection, PyramidsTraits<SE3, C>::getLevel(frame1, data.sensor, 0), patch);
  features::PatternPatch::shiftPattern(projection, pattern);
  SE3 t_w_target = frame1.tWorldAgent();
  SE3 t_w_host = frame2.tWorldAgent();
  double idepth = 0.05;
  Eigen::Vector2d aff_brightness_target = Eigen::Vector2d::Random() * 0.5;
  Eigen::Vector2d aff_brightness_host = Eigen::Vector2d::Random() * 0.5;
  Eigen::Vector<double, ModelDof> intrinsics_parameters_reference =
      data.model->intrinsicsParameters().template cast<double>();
  Eigen::Vector<double, ModelDof> intrinsics_parameters_target =
      data.model->intrinsicsParameters().template cast<double>();

  Eigen::Vector<double, 8> target_state_eps = Eigen::Vector<double, 8>::Zero();
  Eigen::Vector<double, 8> host_state_eps = Eigen::Vector<double, 8>::Zero();

  std::deque<std::unique_ptr<
      LocalFrame<double, SE3, typename test_tools::SolverTestData<SE3>::Model, Pattern::kSize, features::PixelMap, C>>>
      frames;
  std::map<size_t, std::vector<energy::problem::DepthMap>> depths_maps;
  depths_maps[0].push_back(energy::problem::DepthMap(1280, 720));
  depths_maps[0][0].map(projection.cast<int>().x(), projection.cast<int>().y()).idepth = static_cast<Precision>(idepth);
  depths_maps[0][0].map(projection.cast<int>().x(), projection.cast<int>().y()).weight = 1;

  const std::map<size_t, const sensors::calibration::CameraMask&> masks = {
      {data.sensor, data.camera->pyramidOfMasks()[0]}};
  auto pyramids1 = pyramids<C>(frame1);
  frames.push_back(std::make_unique<LocalFrame<double, SE3, typename test_tools::SolverTestData<SE3>::Model,
                                               Pattern::kSize, features::PixelMap, C>>(
      frame1.timestamp(), t_w_host, *pyramids1, masks, depths_maps, Eigen::Vector2<double>::Zero(), 0, *data.model,
      FrameParameterization::kFree));
  auto pyramids2 = pyramids<C>(frame2);
  frames.push_back(std::make_unique<LocalFrame<double, SE3, typename test_tools::SolverTestData<SE3>::Model,
                                               Pattern::kSize, features::PixelMap, C>>(
      frame2.timestamp(), t_w_target, *pyramids2, masks, Eigen::Vector2<double>::Zero(), false, 0, *data.model,
      FrameParameterization::kFree));
  frames[0]->affine_brightness0 = aff_brightness_host;
  frames[1]->affine_brightness0 = aff_brightness_target;

  frames[0]->residuals[std::make_pair(data.sensor, data.sensor)][frames[1]->id].push_back(
      track::PointConnectionStatus::kOk);
  /**
   * we only compare derivative at zero, for points other than on zero derivative compare will fail, as the new function
   * evaluates d f(T boxplus (x + eps)) / d(eps)  while previous estimated d f((T boxplus x) boxplus eps) / d(eps)
   *
   * in practice for small eps this linerization is good enough but simpiler to compute
   */

  auto callback = std::make_unique<BundleAdjustmentPhotometricEvaluationCallback<
      double, SE3, typename test_tools::SolverTestData<SE3>::Model, Pattern::kSize, features::PixelMap, C, true, true>>(
      frames);

  auto analytic_functor = std::make_unique<CostFunctorAnalytic>(
      frames[0]->residuals[std::make_pair(data.sensor, data.sensor)][frames[1]->id][0]);
  CostFunctorAuto* auto_functor_ = new CostFunctorAuto(
      t_w_host.cast<double>(), t_w_target.cast<double>(), aff_brightness_host, aff_brightness_target,
      PyramidsTraits<SE3, C>::getLevel(frame2, data.sensor, 0), pattern.cast<double>(), patch.template cast<double>(),
      data.camera->pyramidOfMasks()[0], data.model->image_size(), data.model->image_size());
  auto auto_functor = std::make_unique<
      ceres::AutoDiffCostFunction<CostFunctorAuto, CostFunctorAuto::residuals_num, 8, 8, 1, ModelDof, ModelDof>>(
      auto_functor_);

  double const* const parameters[3] = {host_state_eps.data(), target_state_eps.data(), &idepth};
  double const* const parameters_autodiff[5] = {host_state_eps.data(), target_state_eps.data(), &idepth,
                                                intrinsics_parameters_reference.data(),
                                                intrinsics_parameters_target.data()};

  Eigen::Matrix<double, Pattern::kSize * C, 1> residuals, residuals_autodiff;
  Eigen::Matrix<double, Pattern::kSize * C, 8, Eigen::RowMajor> d_host_state_eps, d_host_state_eps_autodiff;
  Eigen::Matrix<double, Pattern::kSize * C, 8, Eigen::RowMajor> d_target_state_eps, d_target_state_eps_autodiff;
  Eigen::Matrix<double, Pattern::kSize * C, 1> d_idepth, d_idepth_autodiff;
  Eigen::Matrix<double, Pattern::kSize * C, ModelDof> d_intrinsics_parameters_reference, d_intrinsics_parameters_target;
  double* jacobians[3] = {d_host_state_eps.data(), d_target_state_eps.data(), d_idepth.data()};

  double* jacobians_autodiff[5] = {d_host_state_eps_autodiff.data(), d_target_state_eps_autodiff.data(),
                                   d_idepth_autodiff.data(), d_intrinsics_parameters_reference.data(),
                                   d_intrinsics_parameters_target.data()};

  firstEstimateJacobians_<double, SE3, typename test_tools::SolverTestData<SE3>::Model, Pattern::kSize,
                          features::PixelMap, C>(frames);
  callback->PrepareForEvaluation(true, true);
  analytic_functor->Evaluate(parameters, residuals.data(), jacobians);
  auto_functor->Evaluate(parameters_autodiff, residuals_autodiff.data(), jacobians_autodiff);
  EXPECT_TRUE((residuals - residuals_autodiff).norm() < 1e-5);
  EXPECT_TRUE((d_idepth - d_idepth_autodiff).norm() < 1e-5);
  EXPECT_TRUE((d_host_state_eps - d_host_state_eps_autodiff).norm() < 1e-5);
  EXPECT_TRUE((d_target_state_eps - d_target_state_eps_autodiff).norm() < 1e-5);
}
}  // namespace

TEST(analytical_diff, analytical_diff_one_channel) { testAnalyticalDiff<1>(); }

}  // namespace problem
}  // namespace energy
}  // namespace dsopp
