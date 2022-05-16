#ifndef DSOPP_ENERGY_PROBLEMS_POSE_ALIGNMENT_PRECALCULATED_POSE_ALIGNMENT_HPP_
#define DSOPP_ENERGY_PROBLEMS_POSE_ALIGNMENT_PRECALCULATED_POSE_ALIGNMENT_HPP_

#include "energy/problems/pose_alignment/pose_alignment.hpp"

#include "common/settings.hpp"
#include "common/time/time.hpp"
#include "energy/motion/motion.hpp"
#include "features/camera/pixel_map.hpp"

namespace dsopp::energy::problem {
/** \brief Class that loads known poses from file and used them as solution
 *
 * @tparam Motion motion type
 * @tparam Model camera model type
 * @tparam PatternSize pattern size used in the alignment
 * @tparam Grid2D jacobian or intensities interpolator
 * @tparam C number of channels in the grid
 */
template <energy::motion::Motion Motion, model::Model Model, int PatternSize = 1, int C = 1>
class PrecalculatedPoseAlignment : public PoseAlignment<Motion, Model, PatternSize, features::PixelMap, C> {
 public:
  /**
   * @param timestamp_t_w_agents map of timestamp-t_w_agent pairs
   *
   */
  PrecalculatedPoseAlignment(std::map<time, Motion> &&timestamp_t_w_agents);
  /**
   * Tries to find pose inside ``timestamp_t_w_agents_``
   *
   * @return kZeroCost of PoseAlignment
   */
  Precision solve(const size_t);

  /**
   * clears solver, deletes all frames and poses
   */
  void reset();

  /**
   * Does nothing, as poses for all timestamps should already be here
   *
   * @param timestamp timestamp
   * @param t_w_agent motion from agent to world
   */
  void pushKnownPose(time timestamp, const Motion &t_w_agent);
};
}  // namespace dsopp::energy::problem

#endif  // DSOPP_ENERGY_PROBLEMS_POSE_ALIGNMENT_PRECALCULATED_POSE_ALIGNMENT_HPP_
