#ifndef DSOPP_SRC_TRACKER_DEPTH_DEPTH_ESTIMATOR_HPP_
#define DSOPP_SRC_TRACKER_DEPTH_DEPTH_ESTIMATOR_HPP_

#include <concepts>
#include <functional>

#include "common/settings.hpp"
#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"
#include "features/camera/pattern_patch.hpp"
#include "features/camera/pixel_map.hpp"
#include "sensors/camera_calibration/camera_calibration.hpp"
#include "sensors/camera_calibration/mask/camera_mask.hpp"

#include "track/landmarks/immature_tracking_landmark.hpp"

namespace dsopp {
namespace tracker {
/** \brief Concept for all implementations that estimate depth.
 *
 * All classes that estimate depth of landmarks between two frame have to be arguments to this class.
 * @tparam Estimator class that estimates depth.
 */
template <typename Estimator, class MotionProduct, int C = 1>
concept DepthEstimator =
    requires(const features::PixelMap<C> &target_frame,
             std::vector<std::reference_wrapper<track::landmarks::ImmatureTrackingLandmark>> &reference_landmarks,
             const MotionProduct &t_t_r, const Precision reference_exposure_time,
             const Eigen::Vector2<Precision> &reference_affine_brightness, const Precision target_exposure_time,
             const Eigen::Vector2<Precision> &target_affine_brightness,
             const sensors::calibration::CameraCalibration &calibration,
             const sensors::calibration::CameraMask &camera_mask, const Precision sigma_huber_loss) {
  {
    Estimator::template estimate<MotionProduct, features::PixelMap, energy::model::PinholeCamera<Precision>, C>(
        target_frame, reference_landmarks, t_t_r, reference_exposure_time, reference_affine_brightness,
        target_exposure_time, target_affine_brightness, calibration, camera_mask, sigma_huber_loss)
  }
  ->std::same_as<void>;
};
}  // namespace tracker
}  // namespace dsopp

#endif  // DSOPP_SRC_TRACKER_DEPTH_DEPTH_ESTIMATOR_HPP_
