#ifndef DSOPP_LANDMARKS_ACTIVATOR_HPP
#define DSOPP_LANDMARKS_ACTIVATOR_HPP

#include <deque>
#include <memory>
#include <vector>

#include "common/settings.hpp"
#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"
#include "features/camera/pattern_patch.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"
#include "sensors/camera_calibration/mask/camera_mask.hpp"

namespace dsopp {
namespace track {
template <energy::motion::Motion Motion>
class ActiveOdometryTrack;
}  // namespace track
namespace tracker {
/**
 * Landmarks activator chooses which immature points to remove,
 * which to skip, and which to activate. The activator maintains
 * the sparsity of the projections at a given level.
 * @tparam Motion Motion type
 * @tparam Calibration camera calibration type
 * @tparam Grid2D 2D grid to evaluate data
 * @tparam C number of channels in the grid
 * @tparam REFINE use optimization before activation of the landmark
 */
template <energy::motion::Motion Motion, energy::model::Model Model, template <int> typename Grid2D, int C,
          bool REFINE = false>
class LandmarksActivator {
 public:
  /**
   * creates Landmarks Activator
   * @param calibration camera calibration
   * @param sigma_huber_loss constant for using in the huber norm
   * @param number_of_desired_points desired number of the feature points
   */
  LandmarksActivator(const sensors::calibration::CameraCalibration &calibration, const Precision sigma_huber_loss = 9,
                     const size_t number_of_desired_points = 2000);
  /**
   * activate immature points in the track
   * @param track track to activate immature points
   */
  void activate(track::ActiveOdometryTrack<Motion> &track);

 private:
  /** camera model to reproject landmarks */
  const sensors::calibration::CameraCalibration &calibration_;
  /** min distance between reprojections to maintain the sparsity */
  Precision min_distance_to_neighbor_ = 2;
  /** sigma_huber_loss constant for using in the huber loss function */
  const Precision sigma_huber_loss_;
  /**desired number of the feature points*/
  const size_t number_of_desired_points_;
};

}  // namespace tracker
}  // namespace dsopp

#endif  // DSOPP_LANDMARKS_ACTIVATOR_HPP
