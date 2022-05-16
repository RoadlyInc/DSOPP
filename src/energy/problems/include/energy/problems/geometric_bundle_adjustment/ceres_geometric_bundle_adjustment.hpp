#ifndef DSOPP_ENERGY_PROBLEM_GEOMETRIC_BUNDLE_ADJUSTMENT_CERES_GEOMETRIC_BUNDLE_ADJUSTMENT_HPP_
#define DSOPP_ENERGY_PROBLEM_GEOMETRIC_BUNDLE_ADJUSTMENT_CERES_GEOMETRIC_BUNDLE_ADJUSTMENT_HPP_

#include <vector>

#include <Eigen/Dense>

#include "common/settings.hpp"
#include "energy/problems/geometric_bundle_adjustment/local_frame.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"

namespace dsopp::energy::problem::geometric_bundle_adjustment {
/**
 * struct that have information about which parameters will be fixed during optimization
 */
struct ParameterParameterization {
  /** fix focal length */
  bool fix_focal = true;
  /** fix principal point */
  bool fix_center = true;
  /** fix model specific (e.g. distortion) */
  bool fix_model_specific = true;
  /** fix frame positions */
  bool fix_poses = false;
  /** fix positions of the landmarks */
  bool fix_landmarks = false;
};
/**
 * \brief Geometric bundle adjustment on ceres solver with ``ceres::Jet`` jacobians
 * optimizes reprojection error with huber loss
 *
 * @tparam Calibration camera calibration type
 */
template <model::Model Model>
class CeresGeometricBundleAdjustmentSolver {
 public:
  /**
   * constructor
   * @param image_size, camera_intrinsics camera model parameters
   * @param number_of_threads number of threads
   * @param loss_threshold threshold for huber oss in pixels
   * @param max_solver_time maximum ceres will perform optimizations
   */
  CeresGeometricBundleAdjustmentSolver(const Eigen::Vector<Precision, 2> &image_size,
                                       const Eigen::Vector<Precision, Model::DoF> &camera_intrinsics,
                                       int number_of_threads, double loss_threshold, double max_solver_time = 1e9);
  /**
   * get optimized intrinsic parameters
   *
   * @return camera intrinsic parameters vector
   */
  const Eigen::Vector<double, Model::DoF> &camera_intrinsics() const;

  /**
   * optimize `tAgentWorld` in local frames, `Vector3d *` in local frame's `associated_point_3d`
   *
   * @param local_frames local frame with poses and pointers to 3d points
   *
   * Camera model optimization parameters set:
   * @param parameter_parameterization information about which parameters will be fixed during optimization
   * @return `true` if ceres decided that solution is usable
   */
  bool solve(std::vector<LocalFrame> &local_frames,
             ParameterParameterization parameter_parameterization = ParameterParameterization());

 private:
  /** vector of camera intrinsic parameters */
  Eigen::Vector<double, Model::DoF> camera_intrinsics_;
  /** image size */
  Eigen::Vector<Precision, 2> image_size_;
  /** number used threads in optimization */
  const int number_of_threads_;
  /** huber loss threshold in pixels */
  const double loss_threshold_;
  /** solver maximum time in seconds */
  const double max_solver_time_;
};

}  // namespace dsopp::energy::problem::geometric_bundle_adjustment

#endif  // DSOPP_ENERGY_PROBLEM_GEOMETRIC_BUNDLE_ADJUSTMENT_CERES_GEOMETRIC_BUNDLE_ADJUSTMENT_HPP_
