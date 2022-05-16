#ifndef DSOPP_ENERGY_EPIPOLAR_GEOMETRY_TRIANGULATION_HPP_
#define DSOPP_ENERGY_EPIPOLAR_GEOMETRY_TRIANGULATION_HPP_

#include <Eigen/Dense>

#include "common/settings.hpp"

namespace dsopp::energy::epipolar_geometry {

/**
 * Method to perform Least Squares triangualtion
 *
 * An exact solution of
 * ||alpha * target - beta * (R * reference) - t||^2
 *
 * then final 3d point in `target` frame could be calculated as
 * (alpha * target) + beta * (R * reference) + t
 *  ___________________________________________
 *                       2
 *
 * @param target target bearing vector
 * @param reference reference bearing vector
 * @param t_t_r 3x4 transformation matrix reference to target frame
 * @return triangulated point in TARGET COORDINATE SYSTEM
 */
template <typename Scalar = Precision>
Eigen::Vector3<Scalar> triangulation(const Eigen::Vector3<Scalar> &target, const Eigen::Vector3<Scalar> &reference,
                                     const Eigen::Matrix<Scalar, 3, 4> &t_t_r) {
  Eigen::Vector3<Scalar> b = (t_t_r.template leftCols<3>() * reference).normalized();
  Eigen::Vector3<Scalar> a = target.normalized();
  Eigen::Vector3<Scalar> t = t_t_r.col(3);

  Scalar ab = a.dot(b);
  Scalar at = a.dot(t);
  Scalar bt = b.dot(t);

  Scalar denom = Scalar(1.0) / (Scalar(1.0) - ab * ab);
  Scalar coeff_a = denom * (at - ab * bt);
  Scalar coeff_b = denom * (ab * at - bt);

  return ((coeff_b * b + coeff_a * a + t) / Scalar(2.0));
}

/**
 * Method to perform Least Squares triangulation by few points
 * \sum (I - bearing * bearing.T) * SE3 * point_3d
 *
 * @param bearing_vectors observed projections of the landmark. SHOULD BE NORMALIZED
 * @param t_agent_world_s 3x4 transformation matrices from world to agent
 * @return triangulated point in WORLD COORDINATE SYSTEM
 */
template <typename Scalar>
Eigen::Vector3<Scalar> triangulation(const std::vector<Eigen::Vector3<Scalar>> &bearing_vectors,
                                     const std::vector<Eigen::Matrix<Scalar, 3, 4>> &t_agent_world_s) {
  Eigen::Matrix4<Scalar> A;
  A.setZero();
  for (size_t i = 0; i < bearing_vectors.size(); ++i) {
    const auto &vector = bearing_vectors[i];
    const auto &t_agent_world = t_agent_world_s[i];
    auto Ai = t_agent_world - vector * vector.transpose() * t_agent_world;
    A += Ai.transpose() * Ai;
  }
  return A.jacobiSvd(Eigen::ComputeFullV).matrixV().col(3).hnormalized();
}

}  // namespace dsopp::energy::epipolar_geometry

#endif  // DSOPP_ENERGY_EPIPOLAR_GEOMETRY_TRIANGULATION_HPP_
