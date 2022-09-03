#ifndef DSOPP_NPY_FOLDER_PROVIDER_HPP
#define DSOPP_NPY_FOLDER_PROVIDER_HPP

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
 * \brief class to fetch camera data from ".npy" arrays in the folder
 *
 * Object of this class fetches camera data from specified path in the alphabetical order
 */
class NpyFolderProvider final : public CameraProvider {
 public:
  /**
   * Creating a provider from the path to folder with ".npy" arrays
   *
   * @param path path to folder with ".npy" arrays
   * @param timestamps_file timestamp file path
   * @param start_frame start frame
   * @param end_frame order number of the last frame to process
   * @param timestamps_frame_id timestamp starting frame id (in case if it is not 0)
   * @param convert_to_grayscale ``true`` if conversion to grayscale is needed
   */
  explicit NpyFolderProvider(const std::string &path, const std::string &timestamps_file, size_t start_frame = 0,
                             size_t end_frame = std::numeric_limits<size_t>::max(), size_t timestamps_frame_id = 0,
                             bool convert_to_grayscale = false);
  /**
   * method fetches images from the ".npy" array.
   *
   * @return next image from the array
   */
  std::unique_ptr<CameraDataFrame> nextFrame() override;
  /**
   * method to access queue size.
   *
   * @return number of frames in the folder that have not been processed yet.
   */
  size_t queueSize() override;

  ~NpyFolderProvider() override = default;

 private:
  /**
   * method to fill batch of frames.
   */
  void fillBatch();
  /** Container containing paths to files and corresponding number of frames in the file */
  std::map<std::string, size_t> file_paths_;
  /** Container containing camera frames in frame_id order */
  std::deque<std::unique_ptr<CameraDataFrame>> frame_batch_;
  /** Container containing times */
  std::map<uint64_t, dsopp::common::file_tools::CameraFrameTimes> times_;
  /** frame id in timestamps */
  size_t timestamps_frame_id_;
};

}  // namespace dsopp::sensors::providers

#endif  // DSOPP_NPY_FOLDER_PROVIDER_HPP
