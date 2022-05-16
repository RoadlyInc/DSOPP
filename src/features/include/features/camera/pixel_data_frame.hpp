#ifndef DSOPP_SRC_FEATURES_CAMERA_PIXEL_DATA_FRAME_HPP_
#define DSOPP_SRC_FEATURES_CAMERA_PIXEL_DATA_FRAME_HPP_

#include <memory>
#include <vector>

#include <opencv2/opencv.hpp>

#include "common/settings.hpp"
#include "common/time/time.hpp"

namespace dsopp {
namespace features {
template <int C>
class PixelMap;

/** shortcut for pyramid of images */
using Pyramid = std::vector<PixelMap<1>>;

/**
 * PixelDataFrame contains the pyramid of images
 */
class PixelDataFrame {
 public:
  /** max num of levels in the pyramid */
  static constexpr size_t kMaxPyramidDepth = 5;

  /** shortcut for photometric calibration */
  using PhotometricCalibration = std::array<Precision, 256>;
  /** shortcut for vignetting */
  using Vignetting = cv::Mat;

  /**
   * creates PixelDataFrame from the given image
   * @param image given image
   * @param photometric_calibration photometric calibration
   * @param vignetting vignetting
   * @param levels number of the levels in the pyramid
   */
  PixelDataFrame(const cv::Mat &image, const PhotometricCalibration &photometric_calibration,
                 const Vignetting &vignetting, const size_t levels);

  /**
   * method to get pyramid level of the data
   * @param level level to get
   * @return pyramid level
   */
  const PixelMap<1> &getLevel(size_t level) const;

  /**
   * method to get all pyramids frame the frame
   * @return pyramids
   */
  const Pyramid &pyramids() const;

  /**
   * method to get the size of the pyramid
   * @return size of the pyramid
   */
  size_t size() const;

  ~PixelDataFrame();

  /**
   * move the pyramid from the frame
   * @param pixel_data_frame pixel data frame
   * @return pyramid
   */
  friend Pyramid movePyramidAndDelete(std::unique_ptr<PixelDataFrame> &pixel_data_frame);

 private:
  /** pyramid of images */
  Pyramid pyramid_;
};

Pyramid movePyramidAndDelete(std::unique_ptr<PixelDataFrame> &pixel_data_frame);
}  // namespace features
}  // namespace dsopp

#endif  // DSOPP_SRC_FEATURES_CAMERA_PIXEL_DATA_FRAME_HPP_
