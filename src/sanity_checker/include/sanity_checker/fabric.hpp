#ifndef DSOPP_SANITY_CHECKER_FABRIC_HPP
#define DSOPP_SANITY_CHECKER_FABRIC_HPP

#include <any>
#include <map>
#include <memory>
#include <string>

#include "energy/motion/motion.hpp"

namespace dsopp {
namespace sanity_checker {
template <template <class> class TrackType, energy::motion::Motion Motion>
class SanityChecker;
/**
 * creates SanityChecker with a given configuration
 * @tparam TrackType track type
 * @tparam Motion motion model type
 * @param parameters parameters specific for the SanityChecker
 * @return created sanity checker or nullptr on failure
 */
template <template <class> class TrackType, energy::motion::Motion Motion>
std::unique_ptr<SanityChecker<TrackType, Motion>> create(const std::map<std::string, std::any> &parameters);
}  // namespace sanity_checker
}  // namespace dsopp

#endif  // DSOPP_SANITY_CHECKER_FABRIC_HPP
