#ifndef DSOPP_IMAGE_OUTPUT_INTERFACE_HPP
#define DSOPP_IMAGE_OUTPUT_INTERFACE_HPP

#include <opencv2/opencv.hpp>

namespace dsopp {
namespace output {
/**
 * \brief Output interface for image.
 *
 * An interface to push debug image.
 */
class ImageOutputInterface {
 public:
  /**
   * push image to be displayed
   * @param image to be displayed
   */
  virtual void pushImage(const cv::Mat image) = 0;
  virtual ~ImageOutputInterface() = default;
};
}  // namespace output
}  // namespace dsopp

#endif  // DSOPP_IMAGE_OUTPUT_INTERFACE_HPP
