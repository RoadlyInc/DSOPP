#ifndef DSOPP_DISTROTED_CAMERA_HPP
#define DSOPP_DISTROTED_CAMERA_HPP

#include <Eigen/Dense>
#include <concepts>

#include "common/settings.hpp"

namespace dsopp::energy::model {
template <typename CameraModel>
concept DistortedCamera = requires(const Eigen::Vector3<Precision> &ray, Eigen::Vector2<Precision> &pt,
                                   const Eigen::Vector2<Precision> &pt1, Eigen::Vector3<Precision> &ray1,
                                   const CameraModel &cam) {
  { cam.project(ray, pt) }
  ->std::same_as<bool>;
  { cam.unproject(pt1, ray1) }
  ->std::same_as<bool>;
  { cam.focalX() }
  ->std::same_as<Precision>;
  { cam.focalY() }
  ->std::same_as<Precision>;
};

}  // namespace dsopp::energy::model

#endif  // DSOPP_DISTROTED_CAMERA_HPP
