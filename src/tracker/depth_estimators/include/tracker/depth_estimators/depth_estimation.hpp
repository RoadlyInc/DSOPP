#ifndef DSOPP_DEPTHESTIMATION_H
#define DSOPP_DEPTHESTIMATION_H

#include <functional>
#include <memory>

#include "common/settings.hpp"
#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"
#include "sensors/camera_calibration/mask/camera_mask.hpp"
#include "track/landmarks/immature_tracking_landmark.hpp"
#include "tracker/depth_estimators/depth_estimator.hpp"

namespace dsopp {
namespace features {
class TrackingFeaturesFrame;

template <int C>
class PixelMap;
class PatternPatch;
}  // namespace features
namespace tracker {

/** \brief Depth estimation class.
 *
 * This class implements box (or block) matching algorithm.
 */
class DepthEstimation {
 public:
  /**
   * estimate depth for each feature in landmarks_frame1 using  two pixmaps and transform between them. This method adds
   * landmarks ("features with depth") to landmarks_frame1.
   *
   * @param target_frame pixel maps of the target frame
   * @param reference_landmarks vector of reference camera landmarks
   * @param t_t_r  transform for the second camera relative to the first one (reference to live relative)
   * @param reference_affine_brightness,target_affine_brightness affine brigtensses
   * @param calibration camera calibration
   * @tparam Grid2D 2D grid to evaluate data
   * @tparam Calibration camera calibration
   * @tparam N pattern patch size
   * @param camera_mask camera mask
   * @param sigma_huber_loss constant for using in the huber loss function
   */
  template <energy::motion::MotionProduct MotionProduct, template <int> typename Grid2D, energy::model::Model Model,
            int C = 1>
  static void estimate(
      const Grid2D<C> &target_frame,
      std::vector<std::reference_wrapper<track::landmarks::ImmatureTrackingLandmark>> &reference_landmarks,
      const MotionProduct &t_t_r, const Eigen::Vector2<Precision> &reference_affine_brightness,
      const Eigen::Vector2<Precision> &target_affine_brightness,
      const sensors::calibration::CameraCalibration &calibration, const sensors::calibration::CameraMask &camera_mask,
      const Precision sigma_huber_loss);
  static_assert(DepthEstimator<DepthEstimation, energy::motion::SE3<Precision>>);
};

}  // namespace tracker
}  // namespace dsopp
#endif  // DSOPP_DEPTHESTIMATION_H
