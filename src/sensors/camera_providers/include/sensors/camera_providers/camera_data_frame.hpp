
#ifndef DSOPP_CAMERA_DATA_FRAME_HPP
#define DSOPP_CAMERA_DATA_FRAME_HPP
#include "sensor/data_frame.hpp"
#include "sensors/camera_calibration/undistorter/undistorter.hpp"

#include <memory>
#include <opencv2/opencv.hpp>

namespace dsopp {
namespace sensors {
namespace providers {

/**
 * \brief DataFrame interface.
 *
 * DataFrame is an object that DataProvider sends to the corresponding sensor.
 */
class CameraDataFrame : public DataFrame {
 public:
  /**
   * Creating a camera data frame from frame data and a unique number.
   *
   * @param frame_id unique frame number
   * @param frame_data frame data loaded by data provider
   * @param exposure_time exposure time
   * @param time time when sensor captured data
   */
  CameraDataFrame(const int frame_id, cv::Mat &&frame_data, const Precision exposure_time, const time time);
  /**
   * method to get frame data.
   *
   * @return frame data.
   */
  const cv::Mat &data() const;

  /**
   * method to get exposure time.
   *
   * @return exposure time.
   */
  Precision exposureTime() const;

 private:
  /** The main container which contains frame data. */
  cv::Mat frame_data_;
  /** Exposure time. */
  const Precision exposure_time_;
};
}  // namespace providers
}  // namespace sensors
}  // namespace dsopp
#endif  // DSOPP_CAMERA_DATA_FRAME_HPP
