#include "sensors/camera_providers/image_video_provider.hpp"

#include <fstream>

#include <glog/logging.h>
#include <opencv2/highgui.hpp>

#include "common/file_tools/camera_frame_times.hpp"

namespace dsopp::sensors::providers {
namespace {
void skipFrames(cv::VideoCapture& video_reader, size_t& timestamps_frame_id, size_t frames_num) {
  for (size_t i = 0; i < frames_num; ++i) {
    video_reader.grab();
  }
  timestamps_frame_id += frames_num;
}
}  // namespace

ImageVideoProvider::ImageVideoProvider(const std::string& path, const std::string& timestamps_file, size_t start_frame,
                                       size_t end_frame, size_t timestamps_frame_id, bool convert_to_grayscale)
    : CameraProvider(start_frame, end_frame, convert_to_grayscale),
      video_reader_(path, cv::CAP_FFMPEG),
      current_frame_id_(start_frame),
      timestamps_frame_id_(timestamps_frame_id) {
  if (!video_reader_.isOpened()) {
    LOG(ERROR) << "Could not open video stream " << path;
    throw std::runtime_error("Video file not found");
  }

  number_of_frames_ = static_cast<size_t>(video_reader_.get(cv::CAP_PROP_FRAME_COUNT));

  skipFrames(video_reader_, timestamps_frame_id_, start_frame);

  common::file_tools::readTimes(timestamps_file, times_);
  // read image size
  image_size_ << static_cast<Precision>(video_reader_.get(cv::CAP_PROP_FRAME_WIDTH)),
      static_cast<Precision>(video_reader_.get(cv::CAP_PROP_FRAME_HEIGHT));
}

std::unique_ptr<CameraDataFrame> ImageVideoProvider::nextFrame() {
  if (number_of_frames_processed_++ >= maximum_number_of_frames_) {
    return nullptr;
  }
  if (number_of_frames_ == current_frame_id_) return nullptr;
  cv::Mat image;
  video_reader_.read(image);

  if (convert_to_grayscale_) {
    if (image.type() != CV_8UC1) cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
  }

  if (times_.count(timestamps_frame_id_) == 0) {
    LOG(FATAL) << "There is no timestamp in the frame with id: " << timestamps_frame_id_;
  }

  uint64_t timestamp = times_.at(timestamps_frame_id_).timestamp;
  current_frame_id_++;
  return std::make_unique<CameraDataFrame>(timestamps_frame_id_++, std::move(image),
                                           time(std::chrono::nanoseconds(timestamp)));
}

size_t ImageVideoProvider::queueSize() { return number_of_frames_ - current_frame_id_; }

}  // namespace dsopp::sensors::providers
