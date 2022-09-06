
#ifndef DSOPP_IMAGE_FOLDER_CAMERA_DATA_PROVIDER_HPP
#define DSOPP_IMAGE_FOLDER_CAMERA_DATA_PROVIDER_HPP

#include <memory>

#include "sensors/camera_providers/camera_data_frame.hpp"
#include "sensors/camera_providers/camera_provider.hpp"

namespace dsopp {
namespace common::file_tools {
struct CameraFrameTimes;
}  // namespace common::file_tools
namespace sensors {
namespace providers {
/**
 * \brief class to fetch images from a folder
 *
 * Object of this class fetches images from specified path in the alphabetical order
 */
class ImageFolderProvider final : public CameraProvider {
 public:
  /**
   * Creating a provider from the path to the folder from which images are loaded.
   *
   * @param path path a directory containing images to provide.
   * @param timestamps_file timestamp file path
   * @param batch_size frame batch size
   * @param start_frame order number of the first frame to process
   * @param end_frame order number of the last frame to process
   * @param convert_to_grayscale ``true`` if conversion to grayscale is needed
   */
  explicit ImageFolderProvider(const std::string &path, const std::string &timestamps_file, size_t batch_size,
                               size_t start_frame = 0, size_t end_frame = std::numeric_limits<size_t>::max(),
                               bool convert_to_grayscale = false);
  /**
   * method fetches images from the image folder.
   *
   * @return next image from the image folder
   */
  std::unique_ptr<CameraDataFrame> nextFrame() override;
  /**
   * method to access queue size.
   *
   * @return number of frames in the folder that have not been processed yet.
   */
  size_t queueSize() override;

  ~ImageFolderProvider() override = default;

 private:
  /**
   * method to fill batch of frames.
   */
  void fillBatch();
  /** Size of the batch */
  size_t batch_size_;
  /** Container containing paths to files in frame_id order */
  std::map<uint32_t, std::string> file_paths_;
  /** Container containing camera frames in frame_id order */
  std::deque<std::unique_ptr<CameraDataFrame>> frame_batch_;
  /** Container containing times */
  std::map<uint64_t, common::file_tools::CameraFrameTimes> times_;
};
}  // namespace providers
}  // namespace sensors
}  // namespace dsopp

#endif  // DSOPP_IMAGE_FOLDER_CAMERA_DATA_PROVIDER_HPP
