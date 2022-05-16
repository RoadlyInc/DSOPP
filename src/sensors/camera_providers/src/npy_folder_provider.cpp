#include "sensors/camera_providers/npy_folder_provider.hpp"

#include <filesystem>
#include <fstream>

#include <cnpy.h>
#include <glog/logging.h>

#include "common/file_tools/timestamps.hpp"

namespace dsopp {
namespace sensors {
namespace providers {

namespace {
/**
 * the list of file extensions to read.
 *
 * NpyFolderProvider provides only files with extensions from this list
 */
const std::set<std::string> kNpyExtensions{".npy"};
/**
 * function to checks if a file is a npy file
 *
 * @param extension extension of file
 * @return true if the file is a npy file and false otherwise
 */
bool isNpy(const std::string &extension) { return kNpyExtensions.count(extension) > 0; }
/**
 * function to read all paths to files and number of images in each file in the given directory
 *
 * @param path path to the given directory
 * @param[out] file_paths all paths to images in the given directory
 */
void readFiles(const std::string &path, std::map<std::string, size_t> &file_paths) {
  const auto &directory_iterator = std::filesystem::directory_iterator(path);
  for (const auto &file_iterator : directory_iterator) {
    const auto &extension = file_iterator.path().filename().extension().string();
    if (isNpy(extension)) {
      const auto &file_path = file_iterator.path().string();
      cnpy::NpyArray npy_raw_data = cnpy::npy_load(file_path);
      file_paths[file_path] = npy_raw_data.shape[0];
    }
  }
}
/**
 * skips frames from the queue
 * @param file_paths сontainer containing paths to files and corresponding number of frames in the file
 * @param timestamps_frame_id frame id in timestamps
 * @param frames_num number of frames to skip
 */
void skipFrames(std::map<std::string, size_t> &file_paths, size_t &timestamps_frame_id, size_t frames_num) {
  size_t last_size = file_paths.begin()->second;
  size_t i = last_size;
  timestamps_frame_id += last_size;
  while (not file_paths.empty() && i <= frames_num) {
    file_paths.erase(file_paths.begin());
    last_size = file_paths.begin()->second;
    i += last_size;
    timestamps_frame_id += last_size;
  }
  i -= last_size;
  timestamps_frame_id -= last_size;
  file_paths.begin()->second -= frames_num - i;
  timestamps_frame_id += frames_num - i;
}
}  // namespace

NpyFolderProvider::NpyFolderProvider(const std::string &path, const std::string &timestamps_file, size_t start_frame,
                                     size_t end_frame, size_t timestamps_frame_id, bool convert_to_grayscale)
    : CameraProvider(start_frame, end_frame, convert_to_grayscale), timestamps_frame_id_(timestamps_frame_id) {
  readFiles(path, file_paths_);
  readTimestamps(timestamps_file, timestamps_);
  skipFrames(file_paths_, timestamps_frame_id_, start_frame);
  fillBatch();
  // read image size
  image_size_ << static_cast<Precision>(frame_batch_.front()->data().cols),
      static_cast<Precision>(frame_batch_.front()->data().rows);
}

void NpyFolderProvider::fillBatch() {
  const auto &file = *file_paths_.begin();
  cnpy::NpyArray npy_raw_data = cnpy::npy_load(file.first);
  size_t number_of_frames = npy_raw_data.shape[0];
  size_t start_frame = number_of_frames - file.second;
  for (size_t i = start_frame; i < number_of_frames; i++) {
    auto data = npy_raw_data.as_vec<uint8_t>();
    size_t height = npy_raw_data.shape[1];
    size_t width = npy_raw_data.shape[2];
    cv::Mat frame_data(static_cast<int>(height), static_cast<int>(width), CV_8UC1, &data[i * width * height]);

    if (timestamps_.count(timestamps_frame_id_) == 0) {
      LOG(WARNING) << "There is no timestamp in the frame with id: " << timestamps_frame_id_;
    }

    uint64_t timestamp = timestamps_[timestamps_frame_id_];
    frame_batch_.push_back(std::make_unique<CameraDataFrame>(timestamps_frame_id_++, frame_data.clone(),
                                                             time(std::chrono::nanoseconds(timestamp))));
  }
  file_paths_.erase(file_paths_.begin());
}

std::unique_ptr<CameraDataFrame> NpyFolderProvider::nextFrame() {
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

size_t NpyFolderProvider::queueSize() {
  return std::accumulate(file_paths_.begin(), file_paths_.end(), frame_batch_.size(),
                         [](const std::size_t previous, const auto &element) { return previous + element.second; });
}

}  // namespace providers
}  // namespace sensors
}  // namespace dsopp
