#include "sanity_checker/fabric.hpp"

#include <fstream>

#include <glog/logging.h>

#include "common/fabric_tools/parameters.hpp"
#include "common/file_tools/parsing.hpp"
#include "common/settings.hpp"
#include "energy/motion/se3_motion.hpp"

#include "sanity_checker/sanity_checker.hpp"
#include "track/active_track.hpp"
#include "track/track.hpp"

namespace dsopp {
namespace sanity_checker {
template <template <class> class TrackType, energy::motion::Motion Motion>
std::unique_ptr<SanityChecker<TrackType, Motion>> create(const std::map<std::string, std::any> &parameters) {
  std::string mode;
  if (!common::fabric_tools::readParameter(mode, parameters, "mode")) {
    return nullptr;
  }
  if (mode == "on") {
    std::string type;
    if (!common::fabric_tools::readParameter(type, parameters, "type")) {
      return nullptr;
    }
    if (type == "ackermann") {
      LOG(WARNING) << "contact Roadly INC for this functionality";
      return nullptr;
    } else {
      LOG(ERROR) << "Inappropriate type for sanity checker";
      return nullptr;
    }
  } else {
    LOG(WARNING) << "Sanity checker is disabled";
    return nullptr;
  }
}

template std::unique_ptr<SanityChecker<track::ActiveTrack, energy::motion::SE3<Precision>>> create(
    const std::map<std::string, std::any> &parameters);
template std::unique_ptr<SanityChecker<track::Track, energy::motion::SE3<Precision>>> create(
    const std::map<std::string, std::any> &parameters);

}  // namespace sanity_checker
}  // namespace dsopp
