
#include "test/tools/tum_gt.hpp"

#include <glog/logging.h>
#include <fstream>

#include "common/file_tools/read_tum_poses.hpp"
#include "common/settings.hpp"
#include "energy/motion/se3_motion.hpp"

namespace dsopp {
namespace test_tools {
TumGt::TumGt(std::string file) {
  std::ifstream stream = std::ifstream(file);
  if (!stream.is_open()) {
    LOG(ERROR) << "Can't load gt file " << file;
    return;
  }
  auto data = common::file_tools::readTumPoses<energy::motion::SE3<Precision>>(stream);
  for (auto& p : data) data_.insert({p.first, Sophus::SE3<Precision>(p.second)});
}
const Sophus::SE3<Precision> TumGt::getPose(time ts) const {
  auto found = getData(ts);
  if (!found) {
    LOG(ERROR) << "Pose for " << ts << " was not found";
    return Sophus::SE3<Precision>();
  }
  return *found;
}
}  // namespace test_tools
}  // namespace dsopp
