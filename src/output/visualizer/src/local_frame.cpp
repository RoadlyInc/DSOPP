#include "visualizer/local_frame.hpp"

namespace dsopp {
namespace output {

namespace tools {
const std::optional<Sophus::SE3d> getDebugCamera(const energy::motion::SE3<Precision> &) { return std::nullopt; }
}  // namespace tools
}  // namespace output
}  // namespace dsopp
