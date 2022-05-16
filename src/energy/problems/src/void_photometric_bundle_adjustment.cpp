#include "energy/problems/photometric_bundle_adjustment/void_photometric_bundle_adjustment.hpp"

#include <map>

#include <glog/logging.h>

#include "energy/camera_model/pinhole/pinhole_camera.hpp"
#include "energy/camera_model/pinhole/simple_radial.hpp"
#include "energy/motion/se3_motion.hpp"

#include "energy/problems/photometric_bundle_adjustment/local_frame.hpp"
#include "features/camera/ceres_grid.hpp"
#include "features/camera/pixel_map.hpp"
#include "track/landmarks/active_tracking_landmark.hpp"
#include "track/landmarks/immature_tracking_landmark.hpp"

namespace dsopp::energy::problem {

template <energy::motion::Motion Motion, model::Model Model, int C>
VoidPhotometricBundleAdjustment<Motion, Model, C>::VoidPhotometricBundleAdjustment() {}

template <energy::motion::Motion Motion, model::Model Model, int C>
Precision VoidPhotometricBundleAdjustment<Motion, Model, C>::solve(const size_t) {
  this->updatePointStatuses(1, 20);
  this->frames_.erase(std::remove_if(this->frames_.begin(), this->frames_.end(),
                                     [](auto &frame) { return frame->to_marginalize || frame->is_marginalized; }),
                      this->frames_.end());

  return 0;
}

#define PBAVoidInstantiation(Motion, Model, C) \
  template class VoidPhotometricBundleAdjustment<energy::motion::Motion<Precision>, model::Model<Precision>, C>
PBAVoidInstantiation(SE3, PinholeCamera, 1);
PBAVoidInstantiation(SE3, SimpleRadialCamera, 1);
#undef PBAVoidInstantiation

}  // namespace dsopp::energy::problem
