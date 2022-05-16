#include "feature_based_slam/tracker/estimate_so3xs2.hpp"

#include <glog/logging.h>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>

#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/camera_model/pinhole/simple_radial.hpp"
#include "energy/problems/so3xs2_refinement.hpp"
#include "feature_based_slam/features/correspondence.hpp"
#include "feature_based_slam/features/distinct_feature.hpp"
#include "feature_based_slam/features/distinct_features_frame.hpp"
#include "feature_based_slam/track/landmarks.hpp"

namespace dsopp {
namespace feature_based_slam {
namespace tracker {

template <typename T>
using StdVector = std::vector<T, Eigen::aligned_allocator<T>>;

template <energy::model::Model Model>
std::pair<size_t, size_t> estimateSO3xS2(size_t reference_frame_id, size_t target_frame_id, track::Landmarks &landmarks,
                                         energy::motion::SE3<Precision> &t_t_r, const size_t number_of_threads,
                                         const Model &camera_model, const size_t min_ransac_matches,
                                         const Precision essential_matrix_ransac_threshold,
                                         const Precision reprojection_threshold) {
  std::vector<size_t> matching_indices;

  StdVector<Eigen::Vector3<double>> reference_bearings, target_bearings;

  for (auto landmark_idx : landmarks.getLandmarksFromTwoFrames(reference_frame_id, target_frame_id)) {
    const auto &landmark = landmarks.landmarks()[landmark_idx];

    const auto *reference_point = landmark.projection(reference_frame_id);
    const auto *target_point = landmark.projection(target_frame_id);

    Eigen::Vector3<Precision> reference_ray, target_ray;
    if (camera_model.unproject(reference_point->coordinates(), reference_ray) and
        camera_model.unproject(target_point->coordinates(), target_ray)) {
      reference_bearings.push_back(reference_ray.template cast<double>().normalized());
      target_bearings.push_back(target_ray.template cast<double>().normalized());
      matching_indices.push_back(landmark_idx);
    }
  }

  if (reference_bearings.size() < min_ransac_matches) return {0, matching_indices.size()};

  using RanSacProblem = opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem;

  opengv::relative_pose::CentralRelativeAdapter adapter(reference_bearings, target_bearings);
  auto relposeproblem = std::make_shared<RanSacProblem>(adapter, RanSacProblem::STEWENIUS, false);

  Precision essential_matrix_ransac_threshold_opengv =
      1._p - std::cos(std::atan(essential_matrix_ransac_threshold / camera_model.focalX()));

  opengv::sac::Ransac<RanSacProblem> ransac;
  ransac.sac_model_ = relposeproblem;
  ransac.threshold_ = essential_matrix_ransac_threshold_opengv;

  ransac.computeModel();

  auto refined = ransac.model_coefficients_;
  ransac.sac_model_->optimizeModelCoefficients(ransac.inliers_, ransac.model_coefficients_, refined);
  auto inliers = ransac.inliers_;

  ransac.sac_model_->selectWithinDistance(refined, essential_matrix_ransac_threshold_opengv, inliers);

  t_t_r = energy::motion::SE3<double>(refined.topLeftCorner<3, 3>(), refined.col(3).normalized())
              .template cast<Precision>();

  size_t initial_matches = reference_bearings.size();
  reference_bearings.clear();
  target_bearings.clear();

  // remove outliers
  std::set<size_t> landmark_inliers;
  for (auto index : inliers) {
    landmark_inliers.insert(matching_indices[static_cast<size_t>(index)]);
  }
  for (auto landmark_idx : landmarks.getLandmarksFromTwoFrames(reference_frame_id, target_frame_id)) {
    if (!landmark_inliers.contains(landmark_idx)) {
      landmarks.removeProjectionFromLandmark(landmark_idx, target_frame_id);
    }
  }

  std::vector<Eigen::Vector2d> reference_projections, target_projections;

  for (auto landmark_idx : landmarks.getLandmarksFromTwoFrames(reference_frame_id, target_frame_id)) {
    const auto &landmark = landmarks.landmarks()[landmark_idx];

    const auto *reference_point = landmark.projection(target_frame_id);
    const auto *target_point = landmark.projection(reference_frame_id);

    reference_projections.push_back(
        (reference_point->coordinates() - camera_model.principal_point()).template cast<double>());
    target_projections.push_back(
        (target_point->coordinates() - camera_model.principal_point()).template cast<double>());
  }

  energy::motion::SE3<double> t_t_r_double = t_t_r.template cast<double>();
  auto focal_length = static_cast<double>(camera_model.focalX());
  energy::problem::refineSO3xS2(focal_length, t_t_r_double, reference_projections, target_projections,
                                reprojection_threshold, static_cast<int>(number_of_threads));

  t_t_r = t_t_r_double.template cast<Precision>();

  return {inliers.size(), initial_matches};
}

template std::pair<size_t, size_t> estimateSO3xS2<energy::model::PinholeCamera<Precision>>(
    size_t reference_frame_id, size_t target_frame_id, track::Landmarks &landmarks,
    energy::motion::SE3<Precision> &t_t_r, const size_t number_of_threads,
    const energy::model::PinholeCamera<Precision> &camera_model, const size_t min_ransac_matches,
    const Precision essential_matrix_ransac_threshold, const Precision reprojection_threshold);

template std::pair<size_t, size_t> estimateSO3xS2<energy::model::SimpleRadialCamera<Precision>>(
    size_t reference_frame_id, size_t target_frame_id, track::Landmarks &landmarks,
    energy::motion::SE3<Precision> &t_t_r, const size_t number_of_threads,
    const energy::model::SimpleRadialCamera<Precision> &camera_model, const size_t min_ransac_matches,
    const Precision essential_matrix_ransac_threshold, const Precision reprojection_threshold);
}  // namespace tracker
}  // namespace feature_based_slam
}  // namespace dsopp
