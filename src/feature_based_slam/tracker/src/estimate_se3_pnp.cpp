#include "feature_based_slam/tracker/estimate_se3_pnp.hpp"

#include <glog/logging.h>
#include <opengv/absolute_pose/CentralAbsoluteAdapter.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp>
#include <set>

#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/camera_model/pinhole/simple_radial.hpp"
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
std::pair<size_t, size_t> estimateSE3PnP(size_t frame_id, track::Landmarks &landmarks,
                                         energy::motion::SE3<Precision> &t_r_t, const Model &camera_model,
                                         const Precision pnp_ransac_threshold) {
  StdVector<Eigen::Vector3<double>> reference_bearings, target_3d_points;
  std::vector<size_t> matching_indices;

  for (auto landmark_idx : landmarks.getLandmarksFromFrame(frame_id)) {
    const auto &landmark = landmarks.landmarks()[landmark_idx];

    if (!landmark.position()) continue;

    const auto *projection = landmark.projection(frame_id);

    Eigen::Vector3<Precision> reference_ray;
    if (camera_model.unproject(projection->coordinates(), reference_ray) && landmark.position()) {
      reference_bearings.push_back(reference_ray.template cast<double>().normalized());
      target_3d_points.push_back(landmark.position()->cast<double>());
      matching_indices.push_back(landmark_idx);
    }
  }

  Precision pnp_ransac_threshold_opengv = 1._p - std::cos(std::atan(pnp_ransac_threshold / camera_model.focalX()));

  opengv::absolute_pose::CentralAbsoluteAdapter adapter(reference_bearings, target_3d_points);
  opengv::sac::Ransac<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem> ransac;

  auto absposeproblem = std::make_shared<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem>(
      adapter, opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem::EPNP, false);
  ransac.sac_model_ = absposeproblem;
  ransac.threshold_ = pnp_ransac_threshold_opengv;

  ransac.computeModel();

  auto refined = ransac.model_coefficients_;
  ransac.sac_model_->optimizeModelCoefficients(ransac.inliers_, ransac.model_coefficients_, refined);
  auto inliers = ransac.inliers_;
  ransac.sac_model_->selectWithinDistance(refined, pnp_ransac_threshold_opengv, inliers);

  t_r_t = energy::motion::SE3<double>(refined.topLeftCorner<3, 3>(), refined.col(3)).template cast<Precision>();

  // remove outliers
  std::set<size_t> landmark_inliers;
  for (auto index : inliers) {
    landmark_inliers.insert(matching_indices[static_cast<size_t>(index)]);
  }
  for (auto landmark_idx : landmarks.getLandmarksFromFrame(frame_id)) {
    if (!landmarks.landmarks()[landmark_idx].position()) continue;
    if (!landmark_inliers.contains(landmark_idx)) {
      landmarks.removeProjectionFromLandmark(landmark_idx, frame_id);
    }
  }

  return {ransac.inliers_.size(), matching_indices.size()};
}

template std::pair<size_t, size_t> estimateSE3PnP<energy::model::PinholeCamera<Precision>>(
    size_t frame_id, track::Landmarks &landmarks, energy::motion::SE3<Precision> &t_r_t,
    const energy::model::PinholeCamera<Precision> &model, const Precision pnp_ransac_threshold);
template std::pair<size_t, size_t> estimateSE3PnP<energy::model::SimpleRadialCamera<Precision>>(
    size_t frame_id, track::Landmarks &landmarks, energy::motion::SE3<Precision> &t_r_t,
    const energy::model::SimpleRadialCamera<Precision> &model, const Precision pnp_ransac_threshold);

}  // namespace tracker
}  // namespace feature_based_slam
}  // namespace dsopp
