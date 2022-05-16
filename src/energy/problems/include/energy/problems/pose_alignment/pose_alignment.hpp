#ifndef DSOPP_ENERGY_PROBLEM_PHOTOMETRIC_POSE_ALIGNMENT_HPP_
#define DSOPP_ENERGY_PROBLEM_PHOTOMETRIC_POSE_ALIGNMENT_HPP_

#include "energy/problems/photometric_bundle_adjustment/photometric_bundle_adjustment.hpp"

#include <map>

#include "common/settings.hpp"
#include "common/timestamp_storage/timestamp_storage.hpp"
#include "energy/motion/motion.hpp"

namespace dsopp::energy::problem {

/**
 * \brief Base class for Pose alignment without depth estimation. Works on 2 consecutive frames
 * @tparam Motion motion type
 * @tparam Model camera model type
 * @tparam PatternSize pattern size
 * @tparam Grid2D jacobian evaluator and value interpolator grid type
 * @tparam C number of channels
 */
template <energy::motion::Motion Motion, model::Model Model, int PatternSize = 1,
          template <int> typename Grid2D = features::PixelMap, int C = 1>
class PoseAlignment
    : public PhotometricBundleAdjustment<Precision, Motion, Model, PatternSize, Grid2D, true, false, false, C> {
 public:
  /** Zero cost of the solver */
  static constexpr Precision kZeroCost = -1.0_p;
  /**
   * Solve the photometric bundle adjustment problem.
   *
   * @return return root mean square error in energy per pixel
   */
  virtual Precision solve(const size_t) = 0;

  /**
   * set rotation prior
   */
  virtual void setRotationPrior(const Eigen::Matrix3<Precision> &) {}

  /**
   * clears solver, deletes all frames and poses
   */
  virtual void reset() = 0;

  /**
   * Push known pose from agent to world at ``timestamp``
   * @param timestamp timestamp
   * @param t_w_agent motion from agent to world
   */
  virtual void pushKnownPose(time timestamp, const Motion &t_w_agent) = 0;

  virtual ~PoseAlignment() = default;

 protected:
  /**
   * Constructor to create CeresPhotometricBundleAdjustment solver
   */
  PoseAlignment() {}

  /** contains timestamps and Positions (pose from agent to world) */
  common::timestamp_storage::TimestampStorage<Motion> timestamp_t_w_agents_;
};
}  // namespace dsopp::energy::problem

#endif  // DSOPP_ENERGY_PROBLEM_PHOTOMETRIC_POSE_ALIGNMENT_HPP_
