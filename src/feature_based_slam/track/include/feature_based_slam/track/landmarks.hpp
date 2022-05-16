#ifndef DSOPP_FEATURE_BASED_SLAM_TRACK_LANDMARKS_HPP
#define DSOPP_FEATURE_BASED_SLAM_TRACK_LANDMARKS_HPP

#include <deque>

#include "feature_based_slam/features/distinct_feature.hpp"
#include "feature_based_slam/track/landmark.hpp"

namespace dsopp {
namespace feature_based_slam {
namespace track {

/**
 * Storage with all landmarks in the track. Also this class have quick access to landmarks by the projection.
 */
class Landmarks {
 public:
  /**
   * landmark can be created without position if exists only one projection. It will be triangulated later.
   * @return index of the new landmark
   */
  size_t add();
  /**
   * add landmark with the given position
   * @param position 3d position
   * @return index of the new landmark
   */
  size_t add(const Eigen::Vector3<Precision> &position);
  /**
   * @return all landmarks
   */
  const std::deque<Landmark> &landmarks() const;
  /**
   * Set new position to the landmark with index idx
   * @param idx index of the given landmark
   * @param position new position
   */
  void setPositionOfLandmark(const size_t idx, const Eigen::Vector3<Precision> &position);
  /**
   * Add new projection on the frame to the given landmark. It can be deleted later.
   *
   * @param idx landmarks index
   * @param frame_id id of the frame on with landmark projected
   * @param feature feature on the given frame
   */
  void addProjectionToLandmark(const size_t idx, size_t frame_id, const features::DistinctFeature &feature);
  /**
   * Return vector of all landmarks which have projections on the given frame. It can be empty.
   * @param frame_id id of the frame on with landmark projected
   * @return vector of all landmarks which have projections on the given frame
   */
  const std::vector<size_t> &getLandmarksFromFrame(size_t frame_id);
  /**
   * Return vector of all landmarks which have projections on the two given frames. It can be empty.
   * @param frame_id_1, frame_id_2 id of frames on with landmark projected
   * @return vector of all landmarks which have projections on the given frames
   */
  std::vector<size_t> getLandmarksFromTwoFrames(size_t frame_id_1, size_t frame_id_2);
  /**
   * Erase projection on the frame from the given landmark.
   *
   * @param idx landmarks index
   * @param frame_id id of the frame on witch landmark projected
   */
  void removeProjectionFromLandmark(const size_t idx, size_t frame_id);

 private:
  /** all 3d landmarks in the track */
  std::deque<Landmark> landmarks_;
  /** correspondences between frame id landmark, which have projections on this frame */
  std::map<size_t, std::vector<size_t>> landmarks_on_frame_;
};

}  // namespace track
}  // namespace feature_based_slam
}  // namespace dsopp

#endif  // DSOPP_FEATURE_BASED_SLAM_TRACK_LANDMARKS_HPP
