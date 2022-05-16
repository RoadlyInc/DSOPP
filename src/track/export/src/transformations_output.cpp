#include "track/export/transformations_output.hpp"

namespace dsopp {
namespace test_tools {
std::ostream &operator<<(std::ostream &stream, const Sophus::SE3<Precision> &tum_se3) {
  const auto &t = tum_se3.translation();
  const auto &uq = tum_se3.unit_quaternion();
  stream << t.x() << " " << t.y() << " " << t.z() << " " << uq.x() << " " << uq.y() << " " << uq.z() << " " << uq.w();
  return stream;
}

std::ostream &operator<<(std::ostream &stream, const Sophus::Sim3<Precision> &tum_sim3) {
  const auto &t = tum_sim3.translation();
  const auto &q = tum_sim3.quaternion();
  stream << t.x() << " " << t.y() << " " << t.z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w();
  return stream;
}
}  // namespace test_tools
}  // namespace dsopp
