#ifndef DSOPP_SENSORS_CAMERA_PROVIDER_IMAGE_VIDEO_PROVIDER_HPP
#define DSOPP_SENSORS_CAMERA_PROVIDER_IMAGE_VIDEO_PROVIDER_HPP

#include <memory>
#include <string>

#include <opencv2/opencv.hpp>

#include "sensors/camera_providers/camera_data_frame.hpp"
#include "sensors/camera_providers/camera_provider.hpp"

namespace dsopp::common::file_tools {
struct CameraFrameTimes;
}  // namespace dsopp::common::file_tools

namespace dsopp::sensors::providers {

/**
 * \brief class for providing images from video file
 *
 */
class ImageVideoProvider final : public CameraProvider {
 public:
  /**
   * Creating a provider from the path to the video
   *
   * @param path path to video file
   * @param timestamps_file timestamp file path
   * @param start_frame start frame
   * @param end_frame order number of the last frame to process
   * @param timestamps_frame_id timestamp starting frame id (in case if it is not 0)
   * @param convert_to_grayscale ``true`` if conversion to grayscale is needed
   */
  ImageVideoProvider(const std::string &path, const std::string &timestamps_file, size_t start_frame = 0,
                     size_t end_frame = std::numeric_limits<size_t>::max(), size_t timestamps_frame_id = 0,
                     bool convert_to_grayscale = false);

  /**
   * method fetches images from the image video.
   *
   * @return next image from the video stream
   */
  std::unique_ptr<CameraDataFrame> nextFrame() override;

  /**
   * method to access queue size.
   *
   * @return number of frames in the folder that have not been processed yet.
   */
  size_t queueSize() override;

  ~ImageVideoProvider() override = default;

 private:
  /** Video Capturer */
  cv::VideoCapture video_reader_;
  /** container containing camera frames times in frame_id order */
  std::map<uint64_t, dsopp::common::file_tools::CameraFrameTimes> times_;
  /** current frame index */
  size_t current_frame_id_;
  /** number of frames */
  size_t number_of_frames_;
  /** frame id in timestamps */
  size_t timestamps_frame_id_;
};

}  // namespace dsopp::sensors::providers

#endif  // DSOPP_SENSORS_CAMERA_PROVIDER_IMAGE_VIDEO_PROVIDER_HPP
