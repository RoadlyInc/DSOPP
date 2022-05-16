#ifndef DSOPP_ENERGY_PROBLEM_GEOMETRIC_BUNDLE_ADJUSTMENT_LOCAL_FRAME_FRAME_HPP_
#define DSOPP_ENERGY_PROBLEM_GEOMETRIC_BUNDLE_ADJUSTMENT_LOCAL_FRAME_FRAME_HPP_

#include <vector>

#include <Eigen/Dense>

#include "energy/motion/se3_motion.hpp"

namespace dsopp::energy::problem::geometric_bundle_adjustment {

/**
 * \brief local optimization frame for geometric bundle adjustment
 *
 * stores ``tAgentWorld`` transformation and reference to ``Vector3d`` world points
 * This class should be used with ``CeresGeometricBundleAdjustmentSolver``
 * in order to standardize transformation directions used in optimization
 */
class LocalFrame {
 public:
  /** storage type of pixel observations */
  using ObservedPoints = std::vector<Eigen::Vector2d>;
  /**
   * @param frame_id external frame id
   * @param t_agent_world transformation from world to agent
   */
  LocalFrame(size_t frame_id, const motion::SE3<double> t_agent_world);

  /**
   * add new observation to frame
   * ``point_3d`` should be reference to existing ``Vector3d`` in "stable" like deque container
   *
   * @param pixel_observation pixel observation
   * @param point_3d reference to point in the world
   */
  void pushObservation(const Eigen::Vector2d &pixel_observation, Eigen::Vector3d &point_3d);

  /**
   * @return number of observations
   */
  size_t size() const;

  /**
   * @param idx index
   * @return pair of observation in image plane and reference to 3d point
   */
  std::pair<Eigen::Vector2d, Eigen::Vector3d &> point(size_t idx);

  /**
   * @return frame id
   */
  size_t frameId() const;

  /** transformation fromw world to agent */
  motion::SE3<double> t_agent_world;

 private:
  /** frame id */
  const size_t frame_id_;
  /** detected point on image plane */
  ObservedPoints observed_points_;
  /** associated 3d points in the world */
  std::vector<std::reference_wrapper<Eigen::Vector3d>> associated_point_3d_;
};

}  // namespace dsopp::energy::problem::geometric_bundle_adjustment

#endif  // DSOPP_ENERGY_PROBLEM_GEOMETRIC_BUNDLE_ADJUSTMENT_LOCAL_FRAME_FRAME_HPP_
