
#ifndef DSOPP_CAMERA_PROVIDER_HPP
#define DSOPP_CAMERA_PROVIDER_HPP

#include "sensor/data_provider.hpp"

#include <memory>

#include <Eigen/Dense>

#include "common/settings.hpp"
namespace dsopp {
namespace sensors {
namespace providers {
class CameraDataFrame;
/**
 * \brief Provider interface for Camera sensor.
 *
 * CameraProvider is an object that fetches images.
 */
class CameraProvider : public DataProvider {
 public:
  /**
   * @param start_frame order number of the first frame to process
   * @param end_frame order number of the last frame to process
   * @param convert_to_grayscale ``true`` if conversion to grayscale is needed
   */
  CameraProvider(size_t start_frame = 0, size_t end_frame = std::numeric_limits<size_t>::max(),
                 bool convert_to_grayscale = false)
      : maximum_number_of_frames_(end_frame - start_frame), convert_to_grayscale_(convert_to_grayscale) {}
  /**
   * method fetches images from the image stream.
   *
   * @return next image from the image stream
   */
  virtual std::unique_ptr<CameraDataFrame> nextFrame() = 0;
  /**
   * method to access queue size
   *
   * @return number of frames in the queue
   */
  virtual size_t queueSize() = 0;

  ~CameraProvider() override = default;

  /**
   * method to get the size of the images that would be provided
   *
   * @return image size
   */
  const Eigen::Vector2<Precision> &imageSize() { return image_size_; }

 protected:
  /** maximum number of frames. If not greater than zero all frames will be processed */
  const size_t maximum_number_of_frames_ = std::numeric_limits<size_t>::max();
  /** number of frames processed */
  size_t number_of_frames_processed_ = 0;
  /** ``true`` if conversion to grayscale is needed */
  const bool convert_to_grayscale_;
  /** size of image that would be provided */
  Eigen::Vector2<Precision> image_size_;
};

}  // namespace providers
}  // namespace sensors
}  // namespace dsopp

#endif  // DSOPP_CAMERA_PROVIDER_HPP
