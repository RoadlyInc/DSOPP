
#include "sensors/camera_providers/image_folder_provider.hpp"

#include <filesystem>
#include <fstream>

#include <glog/logging.h>

#include "common/file_tools/camera_frame_times.hpp"

namespace dsopp {
namespace sensors {
namespace providers {

namespace {
/**
 * the list of image extensions to read.
 *
 * ImageFolderProvider provides only files with extensions from this list
 */
const std::set<std::string> kImageExtensions{".jpg", ".bmp", ".png"};
/**
 * function to checks if a file is an image
 *
 * @param extension extension of file
 * @return true if the file is an image and false otherwise
 */
bool isImage(const std::string &extension) { return kImageExtensions.count(extension) > 0; }
/**
 * function to read all paths to images in the given directory
 *
 * @param path path to the given directory
 * @param[out] file_paths all paths to images in the given directory
 */
void readImages(const std::string &path, std::map<uint32_t, std::string> &file_paths) {
  const auto &directory_iterator = std::filesystem::directory_iterator(path);
  for (const auto &file_iterator : directory_iterator) {
    const auto &extension = file_iterator.path().filename().extension().string();
    if (isImage(extension)) {
      const auto &file_path = file_iterator.path().string();
      uint32_t frame_id = static_cast<uint32_t>(std::stoul(file_iterator.path().stem().string()));
      file_paths[frame_id] = file_path;
    }
  }
}

/**
 * skips frames from the queue
 * @param file_paths container containing paths to files in frame_id order
 * @param frame_num number of frames to skip
 */
void skipFrames(std::map<uint32_t, std::string> &file_paths, size_t frames_num) {
  size_t i = 0;
  while (not file_paths.empty() && i < frames_num) {
    file_paths.erase(file_paths.begin());
    i++;
  }
}
}  // namespace

ImageFolderProvider::ImageFolderProvider(const std::string &path, const std::string &timestamps_file, size_t batch_size,
                                         size_t start_frame, size_t end_frame, bool convert_to_grayscale)
    : CameraProvider(start_frame, end_frame, convert_to_grayscale), batch_size_(batch_size) {
  readImages(path, file_paths_);
  common::file_tools::readTimes(timestamps_file, times_);
  skipFrames(file_paths_, start_frame);
  // read image size
  auto frame_data = cv::imread(file_paths_.begin()->second);
  image_size_ << static_cast<Precision>(frame_data.cols), static_cast<Precision>(frame_data.rows);
}

void ImageFolderProvider::fillBatch() {
  size_t num_of_frames = std::min(batch_size_, file_paths_.size());
  auto file_paths_iter = file_paths_.begin();
  for (size_t i = 0; i < num_of_frames; i++) {
    cv::Mat frame_data;
    if (convert_to_grayscale_)
      frame_data = cv::imread(file_paths_iter->second, 0);
    else
      frame_data = cv::imread(file_paths_iter->second);
    size_t frame_id = file_paths_iter->first;
    if (times_.count(frame_id) == 0) {
      LOG(FATAL) << "There is no timestamp in the frame with id: " << frame_id;
    }
    uint64_t timestamp = times_.at(frame_id).timestamp;
    frame_batch_.push_back(
        std::make_unique<CameraDataFrame>(frame_id, std::move(frame_data), time(std::chrono::nanoseconds(timestamp))));
    file_paths_iter++;
  }
  file_paths_.erase(file_paths_.begin(), file_paths_iter);
}

std::unique_ptr<CameraDataFrame> ImageFolderProvider::nextFrame() {
  if (number_of_frames_processed_++ >= maximum_number_of_frames_) {
    return nullptr;
  }
  if (frame_batch_.size() == 0) {
    fillBatch();
  }
  if (queueSize() == 0) {
    return nullptr;
  }
  auto next_frame = std::move(frame_batch_.front());
  frame_batch_.pop_front();
  return next_frame;
}

size_t ImageFolderProvider::queueSize() { return file_paths_.size() + frame_batch_.size(); }

}  // namespace providers
}  // namespace sensors
}  // namespace dsopp
