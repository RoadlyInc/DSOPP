#ifndef DSOPP_TRACK2TUM_EXPORTER_HPP
#define DSOPP_TRACK2TUM_EXPORTER_HPP

#include <string>

#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"

namespace dsopp {
namespace track {
template <energy::motion::Motion Motion>
class OdometryTrack;
}  // namespace track

namespace test_tools {
template <energy::motion::Motion Motion>
void track2TumExporter(const track::OdometryTrack<Motion> &track, const std::string &file);
}  // namespace test_tools
}  // namespace dsopp

#endif  // DSOPP_TRACK2TUM_EXPORTER_HPP
