#include "feature_based_slam/tracker/estimate_so3_inlier_count.hpp"

#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/camera_model/pinhole/simple_radial.hpp"
#include "energy/n_point_solvers/pure_rotation_estimator.hpp"
#include "feature_based_slam/features/correspondence.hpp"
#include "feature_based_slam/features/distinct_feature.hpp"
#include "feature_based_slam/features/distinct_features_frame.hpp"
#include "feature_based_slam/track/landmark.hpp"
#include "feature_based_slam/track/landmarks.hpp"
#include "ransac/ransac.hpp"

namespace dsopp {
namespace feature_based_slam {
namespace tracker {

template <size_t rotationNumberOfSamples, energy::model::Model Model>
std::pair<size_t, size_t> estimateSO3inlierCount(size_t reference_frame_id, size_t target_frame_id,
                                                 track::Landmarks &landmarks, Sophus::SO3<Precision> &rotation_t_r,
                                                 const Model &camera_model, const size_t rotation_ransac_iterations,
                                                 const Precision reprojection_threshold) {
  auto common_landmarks = landmarks.getLandmarksFromTwoFrames(reference_frame_id, target_frame_id);
  Eigen::Matrix<Precision, Eigen::Dynamic, 2> reference_points(common_landmarks.size(), 2),
      target_points(common_landmarks.size(), 2);

  int num_of_points = 0;
  for (const auto &landmark_idx : common_landmarks) {
    const auto &landmark = landmarks.landmarks()[landmark_idx];

    const auto *target_point = landmark.projection(target_frame_id);
    const auto *reference_point = landmark.projection(reference_frame_id);

    reference_points.row(num_of_points) = reference_point->coordinates();
    target_points.row(num_of_points) = target_point->coordinates();
    num_of_points++;
  }

  reference_points.conservativeResize(num_of_points, reference_points.cols());
  target_points.conservativeResize(num_of_points, target_points.cols());

  energy::n_point_solvers::PureRorationSolver<rotationNumberOfSamples, Model, Precision> solver(camera_model);
  auto [inliers, best_solution] =
      ransac::ransac(solver, target_points, reference_points, reprojection_threshold, rotation_ransac_iterations);

  rotation_t_r = *best_solution;

  return {inliers.size(), reference_points.rows()};
}

template std::pair<size_t, size_t> estimateSO3inlierCount<3, energy::model::PinholeCamera<Precision>>(
    size_t reference_frame_id, size_t target_frame_id, track::Landmarks &landmarks,
    Sophus::SO3<Precision> &rotation_t_r, const energy::model::PinholeCamera<Precision> &camera_model,
    const size_t rotation_ransac_iterations, const Precision reprojection_threshold);

template std::pair<size_t, size_t> estimateSO3inlierCount<3, energy::model::SimpleRadialCamera<Precision>>(
    size_t reference_frame_id, size_t target_frame_id, track::Landmarks &landmarks,
    Sophus::SO3<Precision> &rotation_t_r, const energy::model::SimpleRadialCamera<Precision> &camera_model,
    const size_t rotation_ransac_iterations, const Precision reprojection_threshold);
}  // namespace tracker
}  // namespace feature_based_slam
}  // namespace dsopp
