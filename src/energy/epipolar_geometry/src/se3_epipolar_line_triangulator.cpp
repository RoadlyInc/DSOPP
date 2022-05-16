#include "energy/epipolar_geometry/se3_epipolar_line_triangulator.hpp"

#include "energy/motion/se3_motion.hpp"

namespace dsopp::energy::epipolar_geometry {

EpipolarLineTriangulatorSE3::EpipolarLineTriangulatorSE3(const energy::motion::SE3<Precision> &t_r_t,
                                                         const Eigen::Vector2<Precision> &focal_lengths,
                                                         const Eigen::Vector2<Precision> &principal_point,
                                                         const Eigen::Vector2<Precision> &point_reference,
                                                         Precision max_idepth)
    : kMaxIdepth(max_idepth) {
  Eigen::Matrix<Precision, 3, 3> K;
  K << focal_lengths(0), 0, principal_point(0), 0, focal_lengths(1), principal_point(1), 0, 0, 1;
  Kt_ = K * t_r_t.translation();
  bearing_vector_reference_ = K * t_r_t.rotationMatrix() * K.inverse() * point_reference.homogeneous();
  Eigen::Vector2<Precision> direction = (bearing_vector_reference_ + max_idepth * Kt_).hnormalized() -
                                        (bearing_vector_reference_ + kZeroIdepthEps * Kt_).hnormalized();
  use_x_direction_ = std::pow(direction(0), 2) > std::pow(direction(1), 2);
}

Precision EpipolarLineTriangulatorSE3::getInverseDepth(const Eigen::Vector2<Precision> &point_target) const {
  Precision idepth;
  Precision x_divider = Kt_[0] - Kt_[2] * point_target[0];
  Precision y_divider = Kt_[1] - Kt_[2] * point_target[1];
  if (use_x_direction_ or std::abs(y_divider) < kZeroIdepthEps) {
    idepth = (bearing_vector_reference_[2] * point_target[0] - bearing_vector_reference_[0]) / x_divider;
  } else {
    idepth = (bearing_vector_reference_[2] * point_target[1] - bearing_vector_reference_[1]) / y_divider;
  }
  if (std::abs(idepth - kMaxIdepth) < kZeroIdepthEps) {
    idepth = kMaxIdepth;
  }
  if (std::abs(idepth) < kZeroIdepthEps) {
    idepth = 0;
  }
  return idepth;
}

}  // namespace dsopp::energy::epipolar_geometry
