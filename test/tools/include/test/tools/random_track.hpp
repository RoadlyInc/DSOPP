
#ifndef DSOPP_RANDOM_TRACK_HPP
#define DSOPP_RANDOM_TRACK_HPP

#include <memory>

#include "agent/agent_settings.hpp"
#include "common/settings.hpp"
#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"
#include "sanity_checker/sanity_checker.hpp"

namespace dsopp {
namespace track {
template <energy::motion::Motion Motion>
class OdometryTrack;
template <energy::motion::Motion Motion>
class Track;
}  // namespace track
namespace test_tools {
std::unique_ptr<track::OdometryTrack<energy::motion::SE3<Precision>>> randomTrack(int offset = 0);

std::unique_ptr<track::Track<energy::motion::SE3<Precision>>> track();
}  // namespace test_tools
}  // namespace dsopp
#endif  // DSOPP_RANDOM_TRACK_HPP
