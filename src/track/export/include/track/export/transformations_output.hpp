#ifndef DSOPP_TRANSFORMATIONS_OUTPUT_HPP
#define DSOPP_TRANSFORMATIONS_OUTPUT_HPP

#include <fstream>

#include <sophus/se3.hpp>
#include <sophus/sim3.hpp>

#include "common/settings.hpp"

namespace dsopp {
namespace test_tools {
std::ostream &operator<<(std::ostream &stream, const Sophus::SE3<Precision> &tum_se3);

std::ostream &operator<<(std::ostream &stream, const Sophus::Sim3<Precision> &tum_sim3);
}  // namespace test_tools
}  // namespace dsopp

#endif  // DSOPP_TRANSFORMATIONS_OUTPUT_HPP
