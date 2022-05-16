#ifndef DSOPP_SANITY_CHECKER_HPP
#define DSOPP_SANITY_CHECKER_HPP

#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"

namespace dsopp {
namespace sanity_checker {
/**
 * SanityChecker for track sanity checks
 * @tparam TrackType track type
 * @tparam Motion motion model type
 */
template <template <class> class TrackType, energy::motion::Motion Motion>
class SanityChecker {
 public:
  /**
   * method to check track for sanity.
   * @param[in,out] track track
   * @return true if check was passed and false otherwise
   */
  virtual bool check(TrackType<Motion> &track) = 0;

  virtual ~SanityChecker() = default;
};
}  // namespace sanity_checker
}  // namespace dsopp

#endif  // DSOPP_SANITY_CHECKER_HPP
