#ifndef DSOPP_SRC_OUTPUT_VISUALIZER_OUTPUT_INTERFACE_HPP
#define DSOPP_SRC_OUTPUT_VISUALIZER_OUTPUT_INTERFACE_HPP

namespace dsopp {
namespace output {
/**
 * Base class for all Visualizer Interfaces, required to manage ownership
 */
class VisualizerOutputInterface {
 public:
  virtual ~VisualizerOutputInterface() = default;
};
}  // namespace output
}  // namespace dsopp

#endif  // DSOPP_SRC_OUTPUT_VISUALIZER_OUTPUT_INTERFACE_HPP
