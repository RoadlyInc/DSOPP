#ifndef DSOPP_COMPARE_TRACK_HPP
#define DSOPP_COMPARE_TRACK_HPP

#include "agent/agent_settings.hpp"
#include "common/settings.hpp"
#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"
#include "sanity_checker/sanity_checker.hpp"
#include "track/odometry_track.hpp"
#include "track/track.hpp"

namespace dsopp {
namespace track {
template <energy::motion::Motion Motion>
class OdometryTrack;
}
namespace test_tools {

template <energy::motion::Motion Motion>
void compareTrack(const track::Track<Motion> &track, const track::Track<Motion> &track_reloaded, const Precision kEps);

}  // namespace test_tools
}  // namespace dsopp
#endif
