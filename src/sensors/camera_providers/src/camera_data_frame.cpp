#include "sensors/camera_providers/camera_data_frame.hpp"

namespace dsopp {
namespace sensors {
namespace providers {
CameraDataFrame::CameraDataFrame(int frame_id, cv::Mat &&frame_data, const Precision exposure_time, const time time)
    : DataFrame(time, frame_id), frame_data_(std::move(frame_data)), exposure_time_(exposure_time) {}

const cv::Mat &CameraDataFrame::data() const { return frame_data_; }

Precision CameraDataFrame::exposureTime() const { return exposure_time_; }
}  // namespace providers
}  // namespace sensors
}  // namespace dsopp
