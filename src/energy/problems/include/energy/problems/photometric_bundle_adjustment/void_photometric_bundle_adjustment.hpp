#ifndef DSOPP_ENERGY_PROBLEM_PHOTOMETRIC_BUNDLE_ADJUSTMENT_VOID_PHOTOMETRIC_BUNDLE_ADJUSTMENT_HPP_
#define DSOPP_ENERGY_PROBLEM_PHOTOMETRIC_BUNDLE_ADJUSTMENT_VOID_PHOTOMETRIC_BUNDLE_ADJUSTMENT_HPP_

#include "common/pattern/pattern.hpp"
#include "energy/motion/motion.hpp"
#include "energy/problems/photometric_bundle_adjustment/photometric_bundle_adjustment.hpp"
#include "track/frames/active_keyframe.hpp"

namespace dsopp::energy::problem {

/**
 * \brief Solver class, which does nothing
 *
 * This solver is sort of switch, that tunrs off global optimization of poses and idepthes
 */
template <energy::motion::Motion Motion, model::Model Model, int C = 1>
class VoidPhotometricBundleAdjustment : public PhotometricBundleAdjustment<Precision, Motion, Model, Pattern::kSize,
                                                                           features::PixelMap, true, true, true, C> {
 public:
  VoidPhotometricBundleAdjustment();
  /**
   * @return 0
   */
  Precision solve(const size_t);
};

}  // namespace dsopp::energy::problem
#endif  // DSOPP_ENERGY_PROBLEM_PHOTOMETRIC_BUNDLE_ADJUSTMENT_VOID_PHOTOMETRIC_BUNDLE_ADJUSTMENT_HPP_
