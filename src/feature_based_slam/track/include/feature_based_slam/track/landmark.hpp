#ifndef DSOPP_FEATURE_BASED_SLAM_FEATURES_LANDMARK_HPP
#define DSOPP_FEATURE_BASED_SLAM_FEATURES_LANDMARK_HPP

#include <map>

#include <Eigen/Dense>

#include "common/settings.hpp"

namespace dsopp {
namespace feature_based_slam {
namespace features {
class DistinctFeature;
}
namespace track {
/**
 * \brief 3d landmark for the feature based slam
 */
class Landmark {
 public:
  /**
   * landmark can be created without position if exists only one projection. It will be triangulated later.
   */
  Landmark();
  /**
   * create landmark with the given position
   * @param position 3d position
   */
  Landmark(const Eigen::Vector3<Precision> &position);
  /**
   * @return position of the landmark in the world's coordinate system if it exists. Landmark
   * can be temporary without position, if it wasn't triangulated.
   */
  const Eigen::Vector3<Precision> *position() const;
  /**
   * Set new position to the landmark
   * @param position new position
   */
  void setPosition(const Eigen::Vector3<Precision> &position);
  /**
   * Add new projection on the frame. Projection always have to feature with the given index.
   * It can be deleted later.
   *
   * @param frame_id id of the frame on with landmark projected
   * @param feature feature on the given frame
   */
  void addProjection(size_t frame_id, const features::DistinctFeature &feature);
  /**
   * @param frame_id id of the frame on with landmark may be projected
   * @return feature of the landmark on the frame with the given frame_id, if it exists, or nullptr otherwise.
   */
  const features::DistinctFeature *projection(size_t frame_id) const;
  /**
   * Remove projection of the landmark on the frame with the given frame_id.
   * @param frame_id frame id
   */
  void removeProjection(size_t frame_id);

 private:
  /** true if landmark was triangulated and has 3d position */
  bool initialized_;
  /** 3d position in the world's coordinate system */
  Eigen::Vector3<Precision> position_;
  /** Correspondence between frame idx and feature that is the projection of the landmark */
  std::map<size_t, features::DistinctFeature> projections_;
};
}  // namespace track
}  // namespace feature_based_slam
}  // namespace dsopp

#endif  // DSOPP_FEATURE_BASED_SLAM_FEATURES_LANDMARK_HPP
