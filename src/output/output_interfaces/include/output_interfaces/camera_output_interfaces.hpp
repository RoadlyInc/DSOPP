#ifndef DSOPP_OUTPUT_CAMERA_OUTPUT_INTERFACES_HPP
#define DSOPP_OUTPUT_CAMERA_OUTPUT_INTERFACES_HPP

#include "common/settings.hpp"
#include "output_interfaces/image_output_interface.hpp"

namespace dsopp {
namespace output {
/**
 * Store all variants of the output interfaces for the camera.
 */
struct CameraOutputInterfaces {
  /** current tracking frame */
  ImageOutputInterface *current_frame = nullptr;
  /** current keyframe */
  ImageOutputInterface *current_keyframe = nullptr;
};
}  // namespace output
}  // namespace dsopp

#endif  // DSOPP_OUTPUT_CAMERA_OUTPUT_INTERFACES_HPP
