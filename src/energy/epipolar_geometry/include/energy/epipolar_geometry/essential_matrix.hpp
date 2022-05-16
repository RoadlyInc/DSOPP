#ifndef DSOPP_ESSENTIAL_MATRIX_HPP
#define DSOPP_ESSENTIAL_MATRIX_HPP

#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include <vector>

namespace dsopp {
namespace energy {
namespace epipolar_geometry {
/**
 * Convert an essential matrix to four reference to target transformation variants. The algorithm from the paper
 * 'An Efficient Solution to the Five-Point Relative Pose Problem' by David Nist´er is used (Section 3.1).
 * @param essential_matrix
 * @return vector with four transformation variants
 */
std::vector<Sophus::SE3d> essentialMatrixToTransformation(const Eigen::Matrix3d &essential_matrix);

/**
 * Convert reference to target transformation to the essential matrix using equation E = hat(t) * R
 * @param t_t_r reference to target transformation
 * @return essential matrix
 */
Eigen::Matrix3d transformationToEssentialMatrix(const Sophus::SE3d &t_t_r);

}  // namespace epipolar_geometry
}  // namespace energy
}  // namespace dsopp

#endif  // DSOPP_ESSENTIAL_MATRIX_HPP
