#include "energy/n_point_solvers/pure_rotation_estimator.hpp"

#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/camera_model/pinhole/simple_radial.hpp"

namespace dsopp {
namespace energy {
namespace n_point_solvers {

template <size_t kRotationNumberOfSamples, typename Model, typename Scalar>
PureRorationSolver<kRotationNumberOfSamples, Model, Scalar>::PureRorationSolver(const Model &model) : model_(model) {}

template <size_t kRotationNumberOfSamples, typename Model, typename Scalar>
bool PureRorationSolver<kRotationNumberOfSamples, Model, Scalar>::solve(
    const Eigen::Matrix<Scalar, SampleSize, DimensionA> &reference_points,
    const Eigen::Matrix<Scalar, SampleSize, DimensionB> &target_points) {
  Eigen::Matrix<Scalar, SampleSize, DimensionA + 1> reference_bearings, target_bearings;
  Eigen::Vector3<Scalar> reference_bearing, target_bearing;

  for (int i = 0; i < SampleSize; ++i) {
    if (model_.unproject(reference_points.row(i).transpose(), reference_bearing) &&
        model_.unproject(target_points.row(i).transpose(), target_bearing)) {
      reference_bearings.row(i) = reference_bearing.normalized();
      target_bearings.row(i) = target_bearing.normalized();
    } else {
      return false;
    }
  }

  Eigen::Matrix3<Scalar> H = Eigen::Matrix3<Scalar>::Zero();
  for (int i = 0; i < SampleSize; ++i) {
    H += reference_bearings.row(i).transpose() * target_bearings.row(i);
  }

  H /= static_cast<Scalar>(reference_bearings.size());

  auto svd = H.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);

  H = svd.matrixV() * svd.matrixU().transpose();

  solution_ = Sophus::SO3<Scalar>::fitToSO3(H.determinant() * H);
  return true;
}

template <size_t kRotationNumberOfSamples, typename Model, typename Scalar>
std::pair<std::vector<int>,
          std::optional<typename PureRorationSolver<kRotationNumberOfSamples, Model, Scalar>::Solution>>
PureRorationSolver<kRotationNumberOfSamples, Model, Scalar>::findBest(
    const Eigen::Matrix<Scalar, Eigen::Dynamic, DimensionA> &reference_points,
    const Eigen::Matrix<Scalar, Eigen::Dynamic, DimensionB> &target_points, Scalar inlier_threshold) const {
  std::vector<int> inliers;

  for (int i = 0; i < reference_points.rows(); ++i) {
    const Eigen::Vector2<Scalar> &target_2d = target_points.row(i);

    Eigen::Vector3<Scalar> reference_bearing;
    model_.unproject(reference_points.row(i).transpose(), reference_bearing);
    Eigen::Vector2<Scalar> reference_onto_target_2d;
    model_.project(solution_ * reference_bearing, reference_onto_target_2d);

    if ((reference_onto_target_2d - target_2d).norm() < inlier_threshold) {
      inliers.push_back(i);
    }
  }

  return {inliers, solution_};
}

template <size_t kRotationNumberOfSamples, typename Model, typename Scalar>
const typename PureRorationSolver<kRotationNumberOfSamples, Model, Scalar>::Solution &
PureRorationSolver<kRotationNumberOfSamples, Model, Scalar>::solution() const {
  return solution_;
}

template class PureRorationSolver<3, energy::model::PinholeCamera<double>, float>;

template class PureRorationSolver<3, energy::model::PinholeCamera<Precision>, Precision>;
template class PureRorationSolver<3, energy::model::SimpleRadialCamera<Precision>, Precision>;

}  // namespace n_point_solvers
}  // namespace energy
}  // namespace dsopp
