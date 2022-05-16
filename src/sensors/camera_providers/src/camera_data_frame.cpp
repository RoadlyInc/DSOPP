#include "sensors/camera_providers/camera_data_frame.hpp"

namespace dsopp {
namespace sensors {
namespace providers {
CameraDataFrame::CameraDataFrame(int frame_id, cv::Mat &&frame_data, const time time)
    : DataFrame(time, frame_id), frame_data_(std::move(frame_data)) {}

const cv::Mat &CameraDataFrame::data() const { return frame_data_; }
}  // namespace providers
}  // namespace sensors
}  // namespace dsopp
