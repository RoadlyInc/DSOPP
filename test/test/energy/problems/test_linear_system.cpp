#include "energy/problems/photometric_bundle_adjustment/bundle_adjustment_photometric_evaluation_callback.hpp"
#include "energy/problems/photometric_bundle_adjustment/ceres_photometric_bundle_adjustment_add_blocks.hpp"
#include "energy/problems/photometric_bundle_adjustment/eigen_photometric_bundle_adjustment_problem.hpp"
#include "energy/problems/photometric_bundle_adjustment/first_estimate_jacobians.hpp"
#include "energy/problems/photometric_bundle_adjustment/hessian_block_evaluation.hpp"

#include <ceres/jet.h>
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include "common/settings.hpp"
#include "energy/normal_linear_system.hpp"
#include "energy/problems/photometric_bundle_adjustment/evaluate_jacobians.hpp"
#include "features/camera/pixel_map.hpp"
#include "test/tools/solver_test_data.hpp"

namespace dsopp {
namespace energy {
namespace problem {

namespace {
template <energy::motion::Motion Motion, typename Model, int PatternSize, int C>
void buildDenseSystem(
    std::deque<std::unique_ptr<LocalFrame<double, Motion, Model, PatternSize, features::PixelMap, C>>> &frames,
    size_t sensor_id, Eigen::MatrixX<double> &jacobian, Eigen::VectorX<double> &residuals, Precision huber_sigma,
    const std::vector<int> &marginalized_frames = {}) {
  const size_t kBlockSize = Motion::DoF + 2;
  size_t num_residuals = 0;
  size_t num_parameters = 0;
  for (size_t reference_frame_index = 0; reference_frame_index < frames.size(); ++reference_frame_index) {
    auto &reference_frame = *frames[reference_frame_index];
    num_parameters += kBlockSize;
    num_parameters += reference_frame.active_landmarks[sensor_id].size();
    num_residuals += reference_frame.active_landmarks[sensor_id].size() * PatternSize * (frames.size() - 1);
  }
  jacobian.resize(static_cast<int>(num_residuals), static_cast<int>(num_parameters));
  residuals.resize(static_cast<int>(num_residuals));
  jacobian.setZero();
  residuals.setZero();

  int current_landmark = 0;
  int current_residual = 0;
  int landmark_block_start = static_cast<int>(frames.size() * kBlockSize);
  for (size_t reference_frame_index = 0; reference_frame_index < frames.size(); ++reference_frame_index) {
    auto &reference_frame = *frames[reference_frame_index];
    for (size_t landmark_index = 0; landmark_index < reference_frame.active_landmarks[sensor_id].size();
         landmark_index++) {
      for (size_t target_frame_index = 0; target_frame_index < frames.size(); ++target_frame_index) {
        if (std::find(marginalized_frames.begin(), marginalized_frames.end(), target_frame_index) !=
                marginalized_frames.end() &&
            (std::find(marginalized_frames.begin(), marginalized_frames.end(), reference_frame_index) ==
                 marginalized_frames.end() ||
             target_frame_index < reference_frame_index)) {
          // this residual is reprojected to the marginalized frame, so it's dropped from the system;
          continue;
        }
        auto &target_frame = *frames[target_frame_index];
        if (reference_frame.id == target_frame.id) {
          continue;
        }

        if (landmark_index >= reference_frame.residuals[{sensor_id, sensor_id}][target_frame.id].size()) continue;
        auto &residual = reference_frame.residuals[{sensor_id, sensor_id}][target_frame.id][landmark_index];
        Precision huber_weight_sqrt = residual.residuals.norm() > huber_sigma
                                          ? static_cast<Precision>(std::sqrt(huber_sigma / residual.residuals.norm()))
                                          : 1;
        jacobian.template block<PatternSize, kBlockSize>(current_residual,
                                                         static_cast<int>(reference_frame_index * kBlockSize)) =
            residual.d_reference_state_eps * huber_weight_sqrt;
        jacobian.template block<PatternSize, kBlockSize>(current_residual,
                                                         static_cast<int>(target_frame_index * kBlockSize)) =
            residual.d_target_state_eps * huber_weight_sqrt;
        jacobian.template block<PatternSize, 1>(current_residual, landmark_block_start + current_landmark) =
            residual.d_idepth * huber_weight_sqrt;
        residuals.template segment<PatternSize>(current_residual) = residual.residuals * huber_weight_sqrt;
        current_residual += static_cast<int>(kBlockSize);
      }

      current_landmark++;
    }
  }
}
}  // namespace

class LinearSystemTest : public ::testing::Test {
 public:
  using SE3 = motion::SE3<Precision>;
  static const size_t kBlockSize = SE3::DoF + 2;
  static const int kPatternSize = 8;
  static constexpr Precision kHuberSigma = 9.0;
  static const int C = 1;
  test_tools::SolverTestData<SE3> data{{0, 10, 20, 30, 40, 50}, false, 50};
  track::ActiveOdometryTrack<SE3> &odometry_track = data.track.odometryTrack();
  const size_t kNumFrames = odometry_track.keyframes().size();
  void SetUp() {
    for (size_t keyframe_id = 0; keyframe_id < odometry_track.keyframes().size(); keyframe_id++) {
      auto frame = std::make_unique<LocalFrame<Precision, SE3, typename test_tools::SolverTestData<SE3>::Model,
                                               kPatternSize, features::PixelMap, C>>(
          *odometry_track.keyframes()[keyframe_id], 0, *data.model, FrameParameterization::kFree);
      frame->state_eps.head<SE3::DoF>() = Eigen::Vector<Precision, SE3::DoF>::Random() * 1e-3;
      frame->T_w_agent_linearization_point =
          frame->T_w_agent_linearization_point.rightIncrement(-frame->state_eps.head<SE3::DoF>());
      frame->update(*odometry_track.keyframes()[keyframe_id], odometry_track.keyframes()[keyframe_id]->connections());

      auto frame_double = std::make_unique<LocalFrame<double, SE3, typename test_tools::SolverTestData<SE3>::Model,
                                                      kPatternSize, features::PixelMap, C>>(
          *odometry_track.keyframes()[keyframe_id], 0, *data.model, FrameParameterization::kFree);
      frame_double->state_eps = frame->state_eps.cast<double>();
      frame_double->T_w_agent_linearization_point = frame->T_w_agent_linearization_point.cast<double>();
      frame_double->update(*odometry_track.keyframes()[keyframe_id],
                           odometry_track.keyframes()[keyframe_id]->connections());
      frames.push_back(std::move(frame));
      frames_double_precision.push_back(std::move(frame_double));
    }
    firstEstimateJacobians_<Precision, SE3, typename test_tools::SolverTestData<SE3>::Model, Pattern::kSize,
                            features::PixelMap, C>(frames);
    evaluateJacobians<Precision, SE3, typename test_tools::SolverTestData<SE3>::Model, Pattern::kSize,
                      features::PixelMap, C, false, true, true, true, true>(frames, kHuberSigma);
    changeResidualStatuses(frames);

    firstEstimateJacobians_<double, SE3, typename test_tools::SolverTestData<SE3>::Model, Pattern::kSize,
                            features::PixelMap, C>(frames_double_precision);
    evaluateJacobians<double, SE3, typename test_tools::SolverTestData<SE3>::Model, Pattern::kSize, features::PixelMap,
                      C, false, true, true, true, true>(frames_double_precision, kHuberSigma);
    changeResidualStatuses(frames_double_precision);
    buildDenseSystem(frames_double_precision, data.sensor, jacobian_dense, residuals_dense, kHuberSigma);
    num_idepths = jacobian_dense.cols() - static_cast<long>(kBlockSize * kNumFrames);
  }

  std::deque<std::unique_ptr<
      LocalFrame<Precision, SE3, typename test_tools::SolverTestData<SE3>::Model, kPatternSize, features::PixelMap, C>>>
      frames;
  std::deque<std::unique_ptr<
      LocalFrame<double, SE3, typename test_tools::SolverTestData<SE3>::Model, kPatternSize, features::PixelMap, C>>>
      frames_double_precision;
  Eigen::MatrixX<double> jacobian_dense;
  Eigen::VectorX<double> residuals_dense;
  long num_idepths = 0;
};

TEST_F(LinearSystemTest, pose_pose_block) {
  Eigen::MatrixX<double> hessian_pose_pose_gt =
      jacobian_dense.leftCols(static_cast<int>(kNumFrames * kBlockSize)).transpose() *
      jacobian_dense.leftCols(static_cast<int>(kNumFrames * kBlockSize));
  Eigen::VectorX<double> b_pose_pose_gt =
      jacobian_dense.leftCols(static_cast<int>(kNumFrames * kBlockSize)).transpose() * residuals_dense;

  NormalLinearSystem<> system_pose_pose(static_cast<int>(kNumFrames * kBlockSize));
  system_pose_pose.setZero();

  evaluateLinearSystemPosePose(frames, data.sensor, system_pose_pose);

  EXPECT_TRUE(((hessian_pose_pose_gt.template cast<Precision>() - system_pose_pose.H).array() <
               hessian_pose_pose_gt.template cast<Precision>().cwiseAbs().array().abs() * 5e-3 + 1e1)
                  .all());
  EXPECT_TRUE(((b_pose_pose_gt.template cast<Precision>() - system_pose_pose.b).array() <
               b_pose_pose_gt.template cast<Precision>().array().cwiseAbs().abs() * 5e-3 + 1e1)
                  .all());
}

TEST_F(LinearSystemTest, pose_idepth_schur_complement) {
  Eigen::MatrixX<double> hessian = jacobian_dense.transpose() * jacobian_dense;
  auto hessian_idepth = hessian.diagonal().tail(num_idepths).array();
  auto hessian_idepth_inverse = (hessian_idepth != 0).select(1 / hessian_idepth, 1).matrix().asDiagonal();
  Eigen::MatrixX<double> hessian_schur_complement_gt =
      hessian.topRightCorner(static_cast<int>(kNumFrames * kBlockSize), num_idepths) * hessian_idepth_inverse *
      hessian.topRightCorner(static_cast<int>(kNumFrames * kBlockSize), num_idepths).transpose();

  Eigen::VectorX<double> b = jacobian_dense.transpose() * residuals_dense;
  Eigen::VectorX<double> b_schur_complement_gt =
      hessian.topRightCorner(static_cast<int>(kNumFrames * kBlockSize), num_idepths) * hessian_idepth_inverse *
      b.tail(num_idepths);

  NormalLinearSystem<> system_schur_complement(static_cast<int>(kNumFrames * kBlockSize));
  system_schur_complement.setZero();
  evaluateLinearSystemPoseDepthSchurComplement(frames, data.sensor, system_schur_complement);
  EXPECT_TRUE(((b_schur_complement_gt.template cast<Precision>() - system_schur_complement.b).array() <
               b_schur_complement_gt.template cast<Precision>().cwiseAbs().array().abs() * 5e-3 + 1e1)
                  .all());
  EXPECT_TRUE(((hessian_schur_complement_gt.template cast<Precision>() - system_schur_complement.H).array() <
               hessian_schur_complement_gt.template cast<Precision>().cwiseAbs().array().abs() * 5e-3 + 1e1)
                  .all());
}

TEST_F(LinearSystemTest, ceres_jacobian) {
  const Precision kAffineBrightnessLambdaSqrt = 1e6;

  // Add affine brightness prior to the jacobian_dense
  long current_residual = residuals_dense.rows();
  jacobian_dense.conservativeResize(residuals_dense.rows() + int(frames.size()) * 2, jacobian_dense.cols());
  for (size_t reference_frame_index = 0; reference_frame_index < frames.size(); ++reference_frame_index) {
    jacobian_dense.row(current_residual).setZero();
    jacobian_dense.row(current_residual + 1).setZero();
    if (frames[reference_frame_index]->frame_parameterization != FrameParameterization::kFixed) {
      jacobian_dense.template block<2, 2>(current_residual, int(reference_frame_index * kBlockSize + SE3::DoF))
          .diagonal() = Eigen::Vector2d::Constant(kAffineBrightnessLambdaSqrt);
    }
    current_residual += 2;
  }

  // build ceres problem
  auto loss_function = std::make_unique<ceres::HuberLoss>(kHuberSigma);
  auto *ordering = new ceres::ParameterBlockOrdering();

  auto evaluation_callback = std::make_unique<
      BundleAdjustmentPhotometricEvaluationCallback<double, SE3, typename test_tools::SolverTestData<SE3>::Model,
                                                    Pattern::kSize, features::PixelMap, C, false, true>>(
      frames_double_precision);

  ceres::Problem::Options problem_options;
  problem_options.evaluation_callback = evaluation_callback.get();
  problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres::Problem problem(problem_options);

  addFramesToProblem<SE3, typename test_tools::SolverTestData<SE3>::Model, Pattern::kSize, features::PixelMap, true,
                     true, false, C>(
      frames_double_precision, data.sensor, problem, loss_function.get(), ordering,
      Eigen::Vector2d::Constant(kAffineBrightnessLambdaSqrt * kAffineBrightnessLambdaSqrt), true);

  // get jacobians from the ceres
  ceres::CRSMatrix jacobian;
  ceres::Problem::EvaluateOptions evaluate_options;
  problem.Evaluate(evaluate_options, NULL, NULL, NULL, &jacobian);
  Eigen::MatrixX<double> jacobian_ceres(jacobian.num_rows, jacobian.num_cols);
  jacobian_ceres.setZero();
  for (int r = 0; r < jacobian.num_rows; ++r) {
    for (int idx = jacobian.rows[static_cast<unsigned long>(r)]; idx < jacobian.rows[static_cast<unsigned long>(r + 1)];
         ++idx) {
      const int c = jacobian.cols[static_cast<unsigned long>(idx)];
      jacobian_ceres(r, c) = jacobian.values[static_cast<unsigned long>(idx)];
    }
  }
  // deleting part with intrinsic parameters
  jacobian_ceres = jacobian_ceres
                       .rightCols(jacobian_ceres.cols() - static_cast<int>(frames_double_precision.size()) *
                                                              test_tools::SolverTestData<SE3>::Model::DoF)
                       .eval();

  // compare jacobians from the ceres and eigen system
  int number_of_zero_rows = 0;
  for (int i = 0; i < jacobian_dense.rows(); i++) {
    if (jacobian_dense.row(i).norm() < 1e-4) {
      number_of_zero_rows++;
      continue;
    }
    bool found = false;
    for (int j = 0; j < jacobian_ceres.rows(); j++) {
      if ((jacobian_ceres.row(j) - jacobian_dense.row(i)).norm() < jacobian_ceres.row(j).norm() * 1e-4) {
        found = true;
      }
    }
    EXPECT_TRUE(found);
  }
  EXPECT_TRUE(jacobian_dense.rows() == jacobian_ceres.rows() + number_of_zero_rows);
  Eigen::MatrixX<double> hessian = jacobian_dense.transpose() * jacobian_dense;
  Eigen::MatrixX<double> hessian_ceres = jacobian_ceres.transpose() * jacobian_ceres;
  EXPECT_LE((hessian - hessian_ceres).norm(), 1e-2);
}
TEST_F(LinearSystemTest, marginalize_points) {
  size_t frame_size = frames.size();
  NormalLinearSystem<double> system_gt;
  system_gt.H = jacobian_dense.transpose() * jacobian_dense;
  system_gt.b = jacobian_dense.transpose() * residuals_dense;

  NormalLinearSystem<double> system_marginalized(static_cast<int>(frames.size() * kBlockSize));
  Precision energy_marginalized = 0;
  system_marginalized.setZero();
  std::vector<int> marginalized_part(static_cast<size_t>(jacobian_dense.cols()) - frame_size * kBlockSize, 0);
  std::iota(marginalized_part.begin(), marginalized_part.end(), frame_size * kBlockSize);

  auto &landmarks = frames[0]->active_landmarks[0];

  for (auto &landmark : landmarks) {
    landmark.is_marginalized = true;
    landmark.to_marginalize = true;
  }

  system_gt.reduce_system(marginalized_part);

  updateMarginalizedLinearSystem(frames, data.sensor, system_marginalized, energy_marginalized,
                                 Eigen::Vector2<Precision>::Zero(), 0);

  NormalLinearSystem<> system_pose(static_cast<int>(frames.size() * kBlockSize));
  system_pose.setZero();
  evaluateLinearSystemPosePose(frames, data.sensor, system_pose);

  NormalLinearSystem<> system_schur_complement(static_cast<int>(frames.size() * kBlockSize));
  system_schur_complement.setZero();
  evaluateLinearSystemPoseDepthSchurComplement(frames, data.sensor, system_schur_complement);

  NormalLinearSystem<> system_reduced = system_pose + system_marginalized.cast<Precision>() - system_schur_complement;
  system_reduced.b += system_marginalized.cast<Precision>().H * stateEpsStacked(frames);
  EXPECT_TRUE(((system_reduced.H - system_gt.H.template cast<Precision>()).array() <
               system_reduced.H.cwiseAbs().array().abs() * 5e-3 + 1e1)
                  .all());
  EXPECT_TRUE(((system_reduced.b - system_gt.b.template cast<Precision>()).array() <
               system_reduced.b.cwiseAbs().array().abs() * 5e-3 + 1e1)
                  .all());
}

TEST_F(LinearSystemTest, marginalization) {
  size_t frame_size = frames.size();
  NormalLinearSystem<double> system_marginalized(static_cast<int>(frames.size() * kBlockSize));
  Precision energy_marginalized = 0;
  system_marginalized.setZero();
  // perform marginalization
  for (size_t frame_to_marginalize = 0; frame_to_marginalize < frames_double_precision.size() - 2;
       frame_to_marginalize++) {
    size_t marginalized_frames_num = frame_to_marginalize + 1;
    std::vector<int> marginalized_part(
        static_cast<size_t>(jacobian_dense.cols()) - (frame_size - marginalized_frames_num) * kBlockSize, 0);
    std::iota(marginalized_part.begin(),
              marginalized_part.begin() + static_cast<int>(marginalized_frames_num * kBlockSize), 0);
    std::iota(marginalized_part.begin() + static_cast<int>(marginalized_frames_num * kBlockSize),
              marginalized_part.end(), frame_size * kBlockSize);
    Eigen::MatrixX<double> jacobian_dense_reduced;
    Eigen::VectorX<double> residuals_dense_reduced;
    std::vector<int> marginalized_frames(marginalized_frames_num);
    std::iota(marginalized_frames.begin(), marginalized_frames.end(), 0);
    buildDenseSystem(frames_double_precision, data.sensor, jacobian_dense_reduced, residuals_dense_reduced, kHuberSigma,
                     marginalized_frames);

    NormalLinearSystem<double> system_gt;
    system_gt.H = jacobian_dense_reduced.transpose() * jacobian_dense_reduced;
    system_gt.b = jacobian_dense_reduced.transpose() * residuals_dense_reduced;
    frames[0]->to_marginalize = true;
    frames[0]->is_marginalized = true;
    auto &landmarks = frames[0]->active_landmarks[0];

    for (auto &landmark : landmarks) {
      landmark.is_marginalized = true;
      landmark.to_marginalize = true;
    }

    system_gt.reduce_system(marginalized_part);

    updateMarginalizedLinearSystem(frames, data.sensor, system_marginalized, energy_marginalized,
                                   Eigen::Vector2<Precision>::Zero(), 0);

    NormalLinearSystem<> system_pose(static_cast<int>(frames.size() * kBlockSize));
    system_pose.setZero();
    evaluateLinearSystemPosePose(frames, data.sensor, system_pose);

    NormalLinearSystem<> system_schur_complement(static_cast<int>(frames.size() * kBlockSize));
    system_schur_complement.setZero();
    evaluateLinearSystemPoseDepthSchurComplement(frames, data.sensor, system_schur_complement);

    NormalLinearSystem<> system_reduced = system_pose + system_marginalized.cast<Precision>() - system_schur_complement;
    system_reduced.b += system_marginalized.cast<Precision>().H * stateEpsStacked(frames);
    EXPECT_TRUE(((system_reduced.H - system_gt.H.template cast<Precision>()).array() <
                 system_reduced.H.cwiseAbs().array().abs() * 5e-3 + 1e1)
                    .all());
    EXPECT_TRUE(((system_reduced.b - system_gt.b.template cast<Precision>()).array() <
                 system_reduced.b.cwiseAbs().array().abs() * 5e-3 + 1e1)
                    .all());
  }
}
}  // namespace problem
}  // namespace energy
}  // namespace dsopp
